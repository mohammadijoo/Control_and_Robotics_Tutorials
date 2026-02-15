import torch
import torch.nn as nn
import torch.nn.functional as F
import numpy as np
import cv2

# Optional: ROS interface (comment out if not using ROS)
try:
    import rospy
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False


class SmallSegNet(nn.Module):
    def __init__(self, num_classes: int = 4):
        super().__init__()
        # Encoder
        self.conv1 = nn.Conv2d(3, 32, kernel_size=3, padding=1)
        self.conv2 = nn.Conv2d(32, 64, kernel_size=3, padding=1)
        self.pool = nn.MaxPool2d(kernel_size=2, stride=2)
        # Bottleneck
        self.conv3 = nn.Conv2d(64, 128, kernel_size=3, padding=1)
        # Decoder (simple upsampling)
        self.up1 = nn.ConvTranspose2d(128, 64, kernel_size=2, stride=2)
        self.up2 = nn.ConvTranspose2d(64, 32, kernel_size=2, stride=2)
        self.logits = nn.Conv2d(32, num_classes, kernel_size=1)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        # x: (N, 3, H, W) in [0,1]
        x = F.relu(self.conv1(x))
        x = self.pool(x)          # (N, 32, H/2, W/2)
        x = F.relu(self.conv2(x))
        x = self.pool(x)          # (N, 64, H/4, W/4)
        x = F.relu(self.conv3(x)) # (N, 128, H/4, W/4)
        x = F.relu(self.up1(x))   # (N, 64, H/2, W/2)
        x = F.relu(self.up2(x))   # (N, 32, H,   W)
        x = self.logits(x)        # (N, C, H, W)
        return x


def load_model(weights_path: str, num_classes: int) -> SmallSegNet:
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model = SmallSegNet(num_classes=num_classes).to(device)
    state_dict = torch.load(weights_path, map_location=device)
    model.load_state_dict(state_dict)
    model.eval()
    return model


def segment_rgb_image(model: SmallSegNet, rgb: np.ndarray) -> np.ndarray:
    """
    rgb: uint8 array with shape (H, W, 3), BGR or RGB.
    Returns: uint8 mask with shape (H, W), each entry is class index.
    """
    device = next(model.parameters()).device
    img = cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB)
    img_f = img.astype(np.float32) / 255.0
    tensor = torch.from_numpy(img_f.transpose(2, 0, 1)).unsqueeze(0).to(device)
    with torch.no_grad():
        logits = model(tensor)
        probs = F.softmax(logits, dim=1)
        mask = torch.argmax(probs, dim=1).squeeze(0).cpu().numpy().astype(np.uint8)
    return mask


def backproject_to_base(mask: np.ndarray,
                        depth: np.ndarray,
                        K: np.ndarray,
                        T_base_cam: np.ndarray,
                        target_class: int) -> np.ndarray:
    """
    Back-project pixels of target_class into 3D base frame.
    depth: depth image in meters, shape (H, W).
    K: 3x3 camera intrinsics.
    T_base_cam: 4x4 homogeneous transform.
    Returns: array of 3D points, shape (N, 3).
    """
    ys, xs = np.where(mask == target_class)
    if ys.size == 0:
        return np.zeros((0, 3), dtype=np.float32)

    # Build homogeneous pixel coordinates
    ones = np.ones_like(xs, dtype=np.float32)
    pix = np.stack([xs.astype(np.float32),
                    ys.astype(np.float32),
                    ones], axis=0)  # (3, N)

    depths = depth[ys, xs].astype(np.float32)  # (N,)
    K_inv = np.linalg.inv(K)
    # Camera-frame coordinates
    P_cam = (K_inv @ pix) * depths  # (3, N)
    P_cam_h = np.vstack([P_cam, ones])  # (4, N)
    # Transform to base frame
    P_base_h = T_base_cam @ P_cam_h
    P_base = P_base_h[:3, :].T  # (N, 3)
    return P_base


# Optional ROS node wrapper
def run_ros_node(weights_path: str,
                 num_classes: int,
                 target_class: int):
    if not ROS_AVAILABLE:
        raise RuntimeError("ROS and cv_bridge are not available in this environment.")
    rospy.init_node("semantic_segmentation_node")
    bridge = CvBridge()
    model = load_model(weights_path, num_classes)

    K = np.eye(3, dtype=np.float32)
    T_base_cam = np.eye(4, dtype=np.float32)

    pub_mask = rospy.Publisher("/segmentation/mask", Image, queue_size=1)

    def callback(msg: Image):
        rgb = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        mask = segment_rgb_image(model, rgb)
        mask_msg = bridge.cv2_to_imgmsg(mask, encoding="mono8")
        pub_mask.publish(mask_msg)

    sub = rospy.Subscriber("/camera/rgb/image_raw", Image, callback, queue_size=1)
    rospy.spin()


if __name__ == "__main__":
    # Example standalone usage (no ROS)
    dummy_img = np.zeros((480, 640, 3), dtype=np.uint8)
    model = SmallSegNet(num_classes=4)
    model.eval()
    mask = segment_rgb_image(model, dummy_img)
    print("Mask shape:", mask.shape)
      
