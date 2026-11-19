"""Functions extracted unchanged from the existing ERFNet map-export pipeline.
Only file/module placement changed for this three-module handoff."""
import cv2
import numpy as np


def project_probabilities(p, geometry, homography, output_shape=(800,800)):
    p=np.asarray(p,np.float32)
    if p.ndim!=3 or p.shape[0]!=3 or not np.isfinite(p).all() or p.min() < -1e-6 or p.max()>1+1e-6 or not np.allclose(p.sum(0),1,atol=2e-5):
        raise ValueError('Expected finite normalized probabilities (3,H,W)')
    x,y,w,h=(int(geometry[k]) for k in ('x','y','width','height'))
    ow,oh=geometry['original_size']
    if min(w,h,ow,oh)<=0 or x<0 or y<0 or x+w>p.shape[2] or y+h>p.shape[1]:
        raise ValueError('Invalid letterbox geometry')
    # Coordinate transform between pixel centres, not pixel corners.
    transform=np.asarray(homography,np.float64)@np.array([[ow/w,0,(.5-x)*ow/w-.5],[0,oh/h,(.5-y)*oh/h-.5],[0,0,1]])
    if transform.shape!=(3,3) or not np.isfinite(transform).all() or abs(np.linalg.det(transform))<1e-12:
        raise ValueError('Invalid homography')
    support=np.zeros(p.shape[1:],np.float32);support[y:y+h,x:x+w]=1
    size=(output_shape[1],output_shape[0])
    valid=cv2.warpPerspective(support,transform,size,flags=cv2.INTER_LINEAR,borderValue=0)>=1-1e-6
    warped=cv2.warpPerspective(p.transpose(1,2,0),transform,size,flags=cv2.INTER_LINEAR,borderValue=0)
    return warped.transpose(2,0,1),valid
