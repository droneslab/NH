import cv2
import numpy as np
import torch
from PIL import Image
import sys, os, time


# --- R2D2 Scoring Helpers ---
sys.path.append("/home/spottop/yt-nighthawk/r2d2") # point to the directory where r2d2
from tools import common
from nets.patchnet import *
def norm_RGB(image):
    """Convert a PIL image to a normalized tensor."""
    RGB_mean = torch.tensor([0.485, 0.456, 0.406]).view(3, 1, 1)
    RGB_std = torch.tensor([0.229, 0.224, 0.225]).view(3, 1, 1)
    image_tensor = torch.from_numpy(np.array(image)).float() / 255.0
    image_tensor = image_tensor.permute(2, 0, 1)
    image_tensor = (image_tensor - RGB_mean) / RGB_std
    return image_tensor

def get_r2d2_heatmaps(image, net, device):
    """
    Compute the repeatability and reliability heatmaps for the given image.
    """
    image_tensor = norm_RGB(image).unsqueeze(0).to(device)
    with torch.no_grad():
        res = net(imgs=[image_tensor])
    reliability_map = res.get('reliability')[0][0, 0].cpu().numpy()
    repeatability_map = res.get('repeatability')[0][0, 0].cpu().numpy()
    return repeatability_map, reliability_map

def setup_r2d2(checkpoint_path, gpu_indices=None):
    """
    Set up the R2D2 network.
    """
    try:
        iscuda = common.torch_set_gpu(gpu_indices)
    except Exception as e:
        iscuda = True
    device = torch.device('cuda' if iscuda else 'cpu')
    checkpoint = torch.load(checkpoint_path, map_location=device)
    net_cmd = checkpoint['net']
    net = eval(net_cmd)
    net.load_state_dict({k.replace('module.', ''): v for k, v in checkpoint['state_dict'].items()})
    if iscuda:
        net = net.to(device)
    net.eval()
    return net, device

def get_r2d2_score(repeatability, reliability):
    rep_mean = np.mean(repeatability)
    rely_mean = np.mean(reliability)
    score = rep_mean * (rely_mean ** 2)
    return rep_mean, rely_mean, score

def process_image_r2d2(cv2_image, net, device, resize=None):
    """Process a cv2 image through the R2D2 model."""
    
    # Convert BGR (OpenCV format) to RGB
    img_rgb = cv2.cvtColor(cv2_image, cv2.COLOR_BGR2RGB)
    
    # Resize using OpenCV instead of PIL (only if resize is specified)
    if resize is not None:
        img_rgb = cv2.resize(img_rgb, (resize[0], resize[1]), interpolation=cv2.INTER_AREA)
    
    # Get R2D2 heatmaps
    repeatability, reliability = get_r2d2_heatmaps(img_rgb, net, device)
    
    # Compute final R2D2 score
    rep_mean, rely_mean, score = get_r2d2_score(repeatability, reliability)
    
    return score, repeatability, reliability

def compute_score(image, model, device, resize=None):
    """Compute score using the R2D2 model."""
    score, _, _ = process_image_r2d2(image, model, device, resize=resize)
    return score


def setup_r2d2(checkpoint_path, gpu_indices=None):
    """
    Loads the R2D2 model from checkpoint.
    Assumes that the required modules (e.g. tools.common and nets.patchnet) 
    are available in your PYTHONPATH.
    """
    try:
        iscuda = common.torch_set_gpu(gpu_indices)
    except (RuntimeError, ValueError, AttributeError):
        iscuda = True
    device = torch.device('cuda' if iscuda else 'cpu')
    checkpoint = torch.load(checkpoint_path, map_location=device)
    net_cmd = checkpoint['net']
    net = eval(net_cmd)
    net.load_state_dict({k.replace('module.', ''): v for k, v in checkpoint['state_dict'].items()})
    if iscuda:
        net = net.to(device)
    net.eval()
    return net, device