import torch
import torch.nn as nn
import torch.nn.functional as F

# Example image encoder (replace with real vision model, e.g. ViT or ResNet)
class ImageEncoder(nn.Module):
    def __init__(self, embed_dim: int):
        super().__init__()
        self.conv = nn.Conv2d(3, 32, kernel_size=3, stride=2, padding=1)
        self.fc = nn.Linear(32 * 16 * 16, embed_dim)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        # x: (B, 3, H, W)
        h = F.relu(self.conv(x))
        h = F.adaptive_avg_pool2d(h, (16, 16))
        h = h.view(h.size(0), -1)
        return self.fc(h)  # (B, D)

# Example language encoder stub (replace with a real LM encoder)
class TextEncoder(nn.Module):
    def __init__(self, vocab_size: int, embed_dim: int):
        super().__init__()
        self.emb = nn.Embedding(vocab_size, embed_dim)
        self.fc = nn.Linear(embed_dim, embed_dim)

    def forward(self, tokens: torch.Tensor) -> torch.Tensor:
        # tokens: (B, L_text)
        e = self.emb(tokens)                 # (B, L_text, D)
        e = e.mean(dim=1)                    # simple pooling
        return self.fc(e)                    # (B, D)

class SimpleVLATransformer(nn.Module):
    def __init__(self, embed_dim: int, nhead: int, num_layers: int, action_dim: int):
        super().__init__()
        encoder_layer = nn.TransformerEncoderLayer(
            d_model=embed_dim, nhead=nhead, batch_first=True
        )
        self.trf = nn.TransformerEncoder(encoder_layer, num_layers=num_layers)
        self.fc_mu = nn.Linear(embed_dim, action_dim)
        self.fc_logstd = nn.Linear(embed_dim, action_dim)

    def forward(self, seq_tokens: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        # seq_tokens: (B, L_total, D)
        h = self.trf(seq_tokens)             # (B, L_total, D)
        h_action = h[:, -1, :]               # last token as action token
        mu = self.fc_mu(h_action)
        log_std = torch.clamp(self.fc_logstd(h_action), min=-5.0, max=2.0)
        return mu, log_std

class VisionLanguageActionPolicy(nn.Module):
    def __init__(self, vocab_size: int, action_dim: int, embed_dim: int = 256):
        super().__init__()
        self.img_enc = ImageEncoder(embed_dim)
        self.txt_enc = TextEncoder(vocab_size, embed_dim)
        self.state_fc = nn.Linear(14, embed_dim)  # e.g. 7 joint pos + 7 vel
        self.trf = SimpleVLATransformer(embed_dim, nhead=4, num_layers=4,
                                        action_dim=action_dim)

    def forward(self, img, tokens, state) -> tuple[torch.Tensor, torch.Tensor]:
        # img: (B, 3, H, W), tokens: (B, L_text), state: (B, 14)
        v = self.img_enc(img)                        # (B, D)
        l = self.txt_enc(tokens)                     # (B, D)
        s = F.relu(self.state_fc(state))             # (B, D)

        # Create a short sequence: [lang, vision, state, action_query]
        B, D = v.shape
        action_query = torch.zeros(B, 1, D, device=v.device)
        seq = torch.stack([l, v, s], dim=1)          # (B, 3, D)
        seq = torch.cat([seq, action_query], dim=1)  # (B, 4, D)
        mu, log_std = self.trf(seq)
        return mu, log_std

    def sample(self, img, tokens, state) -> torch.Tensor:
        mu, log_std = self.forward(img, tokens, state)
        std = log_std.exp()
        eps = torch.randn_like(std)
        return mu + std * eps

# Example loss for Gaussian policy
def nll_gaussian(mu: torch.Tensor, log_std: torch.Tensor, a: torch.Tensor) -> torch.Tensor:
    # mu, log_std, a: (B, d)
    var = (2.0 * log_std).exp()
    log_prob = -0.5 * ((a - mu) ** 2 / var + 2.0 * log_std + torch.log(torch.tensor(2.0 * 3.141592653589793)))
    return -log_prob.sum(dim=-1).mean()

# ROS 2 integration sketch (node subscribes to camera, state, instruction)
# and publishes joint velocity commands.
#
# NOTE: This is schematic; message types and topics should match your setup.

"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class VLAControlNode(Node):
    def __init__(self, policy: VisionLanguageActionPolicy):
        super().__init__("vla_control_node")
        self.policy = policy.eval()
        self.img = None
        self.state = None
        self.instr_tokens = None

        self.create_subscription(Image, "/camera/image_raw", self.image_cb, 10)
        self.create_subscription(JointState, "/joint_states", self.state_cb, 10)
        self.create_subscription(String, "/instruction", self.instr_cb, 10)

        self.cmd_pub = self.create_publisher(JointTrajectory, "/arm_controller/joint_trajectory", 10)
        self.timer = self.create_timer(0.1, self.control_loop)

    def image_cb(self, msg: Image):
        # TODO: convert msg to torch.Tensor (B, 3, H, W)
        self.img = ...

    def state_cb(self, msg: JointState):
        # TODO: convert to torch.Tensor (B, 14)
        self.state = ...

    def instr_cb(self, msg: String):
        # TODO: tokenize msg.data to ids
        self.instr_tokens = ...

    @torch.no_grad()
    def control_loop(self):
        if self.img is None or self.state is None or self.instr_tokens is None:
            return
        a = self.policy.sample(self.img, self.instr_tokens, self.state)  # (B, d)

        traj = JointTrajectory()
        # fill trajectory based on a
        self.cmd_pub.publish(traj)
"""
      
