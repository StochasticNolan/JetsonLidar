# Pole Navigation - Minimal Deployment

Single-script pole navigation with LiDAR-camera fusion for Jetson.

## Quick Start

```bash
# Clone on Jetson
git clone <your-repo-url>
cd pole_nav_minimal

# Setup
./scripts/setup.sh

# Run
./scripts/run.sh
```

## What's Included

- `fusion.py` - Main fusion script (camera + LiDAR)
- `config/settings.yaml` - Configuration
- `scripts/setup.sh` - One-time setup
- `scripts/run.sh` - Launch script
- `requirements.txt` - Python dependencies

## Configuration

Edit `config/settings.yaml`:

```yaml
camera:
  device: 0                    # Camera device

lidar:
  enabled: true
  ip: "192.168.1.10"          # Your LiDAR IP
  port: 2368

yolo:
  model: "models/best.pt"     # Your model path
  confidence: 0.3

output:
  save_video: true
  video_path: "/data/output.mp4"
```

## Directory Structure

```
pole_nav_minimal/
├── fusion.py              # Main script
├── config/
│   └── settings.yaml      # Configuration
├── models/
│   └── best.pt           # YOLO model (copy yours here)
├── scripts/
│   ├── setup.sh          # Setup
│   └── run.sh            # Launch
└── requirements.txt
```

## Transfer Your Model

```bash
# Copy your YOLO model to the repo
cp /path/to/your/best.pt pole_nav_minimal/models/

# Commit
git add models/best.pt
git commit -m "Add pole detection model"
git push
```

## On Jetson

```bash
# Clone
git clone <repo-url>
cd pole_nav_minimal

# Setup (one time)
./scripts/setup.sh

# Run
./scripts/run.sh
```

## Outputs

- Live window with detections
- Video saved to `/data/output.mp4`
- Screenshots on 's' key

## Controls

- `q` - Quit
- `p` - Pause
- `s` - Screenshot

## System Requirements

- Python 3.8+
- OpenCV
- CUDA (for Jetson GPU acceleration)
- 4GB+ RAM

## Troubleshooting

**Camera not found**
```bash
ls /dev/video*
# Update camera device in settings.yaml
```

**LiDAR timeout**
```bash
ping 192.168.1.10
# Check IP in settings.yaml
```

**YOLO model missing**
```bash
# Copy your model
cp best.pt models/
```

## License

MIT
