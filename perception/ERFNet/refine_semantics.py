"""Automatic appearance-guided refinement of ERFNet scores, without GT labels.

This is an arena-specific postprocessor, not a new neural network prediction.
No environment name, hand-drawn repair polygon, or ground-truth class map is an
input. A separate valid-region mask may be supplied by the mapping pipeline.
"""
import cv2
import numpy as np

DEFAULTS = {
    'yellow_hue_min': 20, 'yellow_hue_max': 42,
    'yellow_saturation_min': 80, 'yellow_value_min': 85,
    'pink_hue_min': 140, 'pink_hue_max': 179,
    'pink_saturation_min': 65, 'pink_value_min': 55,
    'blue_hue_min': 85, 'blue_hue_max': 135,
    'object_saturation_min': 75, 'object_value_min': 70,
    'component_min_area_at_800': 40,
    'seed_min_pixels_at_800': 3,
    'rail_seed_probability': .25,
    'obstacle_seed_probability': .35,
    'closing_kernel_at_800': 3,
    'dark_rail_bridge_length_at_800': 31,
    'dark_rail_saturation_max': 100,
    'dark_rail_value_max': 110,
    'dark_rail_probability_min': .05,
    'dark_rail_growth_radius_at_800': 0,
    'blue_seed_probability': .35,
    'blue_component_max_area_at_800': 6000,
    'blue_component_max_aspect': 4.,
}


def refine(rgb, probabilities, valid=None, settings=None):
    cfg = dict(DEFAULTS)
    if settings is not None: cfg.update(settings)
    rgb = np.asarray(rgb, np.uint8)
    p = np.asarray(probabilities, np.float32)
    if p.shape != (3, *rgb.shape[:2]) or rgb.shape[2] != 3:
        raise ValueError('RGB/probability shape mismatch')
    if not np.isfinite(p).all() or p.min() < 0 or p.max() > 1.00001:
        raise ValueError('Invalid probabilities')
    if valid is None: valid = np.ones(rgb.shape[:2], bool)
    valid = np.asarray(valid, bool)
    if valid.shape != rgb.shape[:2]: raise ValueError('Invalid support shape')
    baseline = p.argmax(0).astype(np.uint8)
    result = baseline.copy()
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    h,s,v = (hsv[:,:,i] for i in range(3))
    scale = max(rgb.shape[:2]) / 800.
    minimum_area = max(8, round(cfg['component_min_area_at_800'] * scale**2))
    min_seed = max(2, round(cfg['seed_min_pixels_at_800'] * scale**2))
    k = max(3, int(round(cfg['closing_kernel_at_800'] * scale)) | 1)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k,k))
    yellow=(h>=cfg['yellow_hue_min'])&(h<=cfg['yellow_hue_max'])&(s>=cfg['yellow_saturation_min'])&(v>=cfg['yellow_value_min'])
    pink=(h>=cfg['pink_hue_min'])&(h<=cfg['pink_hue_max'])&(s>=cfg['pink_saturation_min'])&(v>=cfg['pink_value_min'])
    blue=(h>=cfg['blue_hue_min'])&(h<=cfg['blue_hue_max'])&(s>=cfg['object_saturation_min'])&(v>=cfg['object_value_min'])
    red=((h<=10)|(h>=170))&(s>=cfg['object_saturation_min'])&(v>=cfg['object_value_min'])
    records=[]

    def grow(colour, class_id, seed_threshold, name, compact=False):
        mask=cv2.morphologyEx((colour&valid).astype(np.uint8),cv2.MORPH_CLOSE,kernel)
        mask[~valid]=0
        count,components,stats,_=cv2.connectedComponentsWithStats(mask,8)
        accepted=0;added=0
        for index in range(1,count):
            if stats[index,cv2.CC_STAT_AREA]<minimum_area: continue
            if compact:
                width,height=stats[index,cv2.CC_STAT_WIDTH],stats[index,cv2.CC_STAT_HEIGHT]
                if max(width,height)/min(width,height)>cfg['blue_component_max_aspect']: continue
                if stats[index,cv2.CC_STAT_AREA]>cfg['blue_component_max_area_at_800']*scale**2: continue
            region=components==index
            # Model evidence must support the intended class in the component.
            seeds=region&(p[class_id]>=seed_threshold)&(p[class_id]>=p[3-class_id])
            if int(seeds.sum())<min_seed: continue
            # Never overwrite the other already predicted occupied class.
            fill=region&(result==0)
            result[fill]=class_id;added+=int(fill.sum());accepted+=1
        records.append({'cue':name,'accepted_components':accepted,'free_pixels_relabelled':added})

    grow(yellow,1,cfg['rail_seed_probability'],'yellow rail')
    grow(pink,1,cfg['rail_seed_probability'],'pink rail panel')
    grow(blue,2,cfg['blue_seed_probability'],'blue object',compact=True)
    grow(red,2,cfg['obstacle_seed_probability'],'red object')
    # Top-down rails are approximately horizontal/vertical. Bridge a bounded
    # short interruption only where the photograph contains dark neutral rail
    # pixels and ERFNet retains some blocked evidence. A long free region, a
    # blue floor stripe, or a gap without rail seeds is not filled by this rule.
    length=max(3,round(cfg['dark_rail_bridge_length_at_800']*scale)|1)
    rail=(result==1).astype(np.uint8)
    bridge=np.zeros_like(valid)
    for shape in ((length,3),(3,length)):
        closed=cv2.morphologyEx(rail,cv2.MORPH_CLOSE,cv2.getStructuringElement(cv2.MORPH_RECT,shape))
        bridge|=closed>0
    dark=(s<=cfg['dark_rail_saturation_max'])&(v<=cfg['dark_rail_value_max'])
    fill=bridge&dark&(p[1]>=cfg['dark_rail_probability_min'])&(result==0)&valid
    result[fill]=1
    records.append({'cue':'image-supported short dark-rail bridge','free_pixels_relabelled':int(fill.sum())})
    radius=round(cfg['dark_rail_growth_radius_at_800']*scale)
    if radius>0:
        neighbourhood=cv2.dilate((result==1).astype(np.uint8),cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(2*radius+1,2*radius+1)))>0
        fill=neighbourhood&dark&(p[1]>=cfg['dark_rail_probability_min'])&(result==0)&valid
        result[fill]=1
        records.append({'cue':'bounded dark rail-edge recovery','free_pixels_relabelled':int(fill.sum())})
    # Only close short gaps. No unrestricted filling of enclosed free regions.
    for cls in (1,2):
        closed=cv2.morphologyEx((result==cls).astype(np.uint8),cv2.MORPH_CLOSE,kernel)>0
        fill=closed&(result==0)&valid
        result[fill]=cls
        records.append({'cue':f'class {cls} small-gap closing','free_pixels_relabelled':int(fill.sum())})
    result[~valid]=255
    return result, {'settings':cfg,'operations':records,
                    'ground_truth_classes_used':False,'manual_repair_polygons_used':False}
