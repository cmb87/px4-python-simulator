#!/bin/sh

APPIMAGE="./flightgear-2024.1.5-linux-amd64.AppImage"
AIRCRAFT_PATH=${AIRCRAFT_PATH_ROOT:-$(dirname "$0")}
FG_WIDTH=${WIDTH:-1920}
FG_HEIGHT=${HEIGHT:-1080}

# set lonlat manually so we spawn in the correct timezone
nice "$APPIMAGE" \
  --native-fdm=socket,in,30,,5503,udp \
  --fdm=external \
  --aircraft=CameraRascal \
  --fg-aircraft="${AIRCRAFT_PATH}/aircraft" \
  --lat=48.35386539065191 \
  --lon=11.78159133408772 \
  --altitude=447 \
  --geometry="${FG_WIDTH}x${FG_HEIGHT}" \
  --bpp=32 \
  --max-fps=30 \
  --disable-hud-3d \
  --disable-sound \
  --disable-fullscreen \
  --timeofday=morning \
  --wind=0@0 \
  --prop:/scenery/use-vpb=true \
  --fov=67 \
  --enable-terrasync \
  "$@"