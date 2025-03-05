
# Build dockers

## Build for ARM
from project root

### Runtime

```bash title="build"
docker  buildx build --platform linux/arm64 -t gst_stream/arm:runtime -f .devcontainer/Dockerfile.arm.runtime .
```

```bash title="usage"
docker run --platform=linux/arm64 --rm -t gst_stream/arm:runtime uname -m 
```