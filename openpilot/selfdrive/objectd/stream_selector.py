ROAD_STREAM_NAME = "road"
WIDE_STREAM_NAME = "wide"
WIDE_CAM_MAX_SPEED = 10.0
ROAD_CAM_MIN_SPEED = 15.0


def select_object_stream(current_stream: str, experimental_mode: bool, v_ego: float) -> str:
  if current_stream not in (ROAD_STREAM_NAME, WIDE_STREAM_NAME):
    current_stream = ROAD_STREAM_NAME
  if not experimental_mode:
    return ROAD_STREAM_NAME
  if v_ego < WIDE_CAM_MAX_SPEED:
    return WIDE_STREAM_NAME
  if v_ego > ROAD_CAM_MIN_SPEED:
    return ROAD_STREAM_NAME
  return current_stream
