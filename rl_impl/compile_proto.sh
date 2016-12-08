PROTO_SRC_DIR=proto
DST_DIR=build
AGENT_DIR=/home/silence/dragon_rl/dragon_ws/src/dragon_agent
# Hack to compile directly into src folders for now
CPP_OUT_DIR=$AGENT_DIR/include/gps/proto
PROTO_BUILD_DIR=$DST_DIR/$PROTO_SRC_DIR
PY_PROTO_BUILD_DIR=python/gps/proto

mkdir -p "$PROTO_BUILD_DIR"
mkdir -p "$PY_PROTO_BUILD_DIR"
touch $PY_PROTO_BUILD_DIR/__init__.py

mkdir -p "$CPP_OUT_DIR"
protoc -I=$PROTO_SRC_DIR --cpp_out=$CPP_OUT_DIR $PROTO_SRC_DIR/gps.proto
protoc -I=$PROTO_SRC_DIR --python_out=$PY_PROTO_BUILD_DIR $PROTO_SRC_DIR/gps.proto

echo "Done"
