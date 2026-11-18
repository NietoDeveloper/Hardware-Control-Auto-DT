#include "sdt/media/media_router.hpp"

namespace sdt {

void MediaRouter::pump() {
  if (sink_ == nullptr) return;
  constexpr int kMaxFramesPerSourcePerPass = 4;
  MediaFrame frame;
  for (IMediaSource* src : sources_) {
    for (int i = 0; i < kMaxFramesPerSourcePerPass; ++i) {
      if (!src->poll(frame)) break;
      ++stats_.frames_in;
      const std::uint8_t id = frame.stream_id;
      const bool video = frame.kind == MediaKind::Video;

      if (video && wait_key_[id] && !frame.keyframe) {  // undecodable without its reference
        ++stats_.frames_dropped;
        continue;
      }
      const bool keyframe = frame.keyframe;
      if (sink_->publish(std::move(frame))) {
        ++stats_.frames_out;
        if (keyframe) wait_key_[id] = false;
      } else {
        ++stats_.frames_dropped;
        if (video) {
          wait_key_[id] = true;
          src->requestKeyframe();
          ++stats_.keyframe_requests;
        }
      }
    }
  }
}

}  // namespace sdt
