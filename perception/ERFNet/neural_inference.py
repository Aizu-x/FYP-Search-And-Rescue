"""Fixed 640-pixel ERFNet inference with explicit source orientation."""
import time
import numpy as np
import cv2
import torch
from PIL import Image,ImageOps


def infer(model,path,size=640,quarter_turns=0):
    with Image.open(path) as im:image=ImageOps.exif_transpose(im).convert('RGB')
    original=image.size
    if quarter_turns:image=image.rotate(90*quarter_turns,expand=True)
    w,h=image.size;s=min(size/w,size/h);nw,nh=round(w*s),round(h*s);x,y=(size-nw)//2,(size-nh)//2
    canvas=Image.new('RGB',(size,size),(127,127,127));canvas.paste(image.resize((nw,nh),Image.Resampling.BILINEAR),(x,y))
    tensor=torch.from_numpy((np.asarray(canvas,np.float32)/255).copy()).permute(2,0,1)[None]
    t=time.perf_counter()
    with torch.inference_mode():p=model(tensor).softmax(1)[0].numpy()
    seconds=time.perf_counter()-t;p=p[:,y:y+nh,x:x+nw]
    if quarter_turns:p=np.rot90(p,-quarter_turns,axes=(1,2)).copy()
    ow,oh=original;s=min(640/ow,640/oh);ew,eh=round(ow*s),round(oh*s);ex,ey=(640-ew)//2,(640-eh)//2
    content=cv2.resize(p.transpose(1,2,0),(ew,eh),interpolation=cv2.INTER_LINEAR).transpose(2,0,1)
    out=np.zeros((3,640,640),np.float32);out[0]=1;out[:,ey:ey+eh,ex:ex+ew]=content
    return out,{'x':ex,'y':ey,'width':ew,'height':eh,'original_size':list(original),
                'network_size':size,'quarter_turns_ccw':quarter_turns,'model_forward_seconds':seconds}
