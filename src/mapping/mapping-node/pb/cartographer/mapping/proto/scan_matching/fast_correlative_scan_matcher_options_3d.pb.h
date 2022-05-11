// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: cartographer/mapping/proto/scan_matching/fast_correlative_scan_matcher_options_3d.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_cartographer_2fmapping_2fproto_2fscan_5fmatching_2ffast_5fcorrelative_5fscan_5fmatcher_5foptions_5f3d_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_cartographer_2fmapping_2fproto_2fscan_5fmatching_2ffast_5fcorrelative_5fscan_5fmatcher_5foptions_5f3d_2eproto

#include <limits>
#include <string>

#include <google/protobuf/port_def.inc>
#if PROTOBUF_VERSION < 3018000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers. Please update
#error your headers.
#endif
#if 3018000 < PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers. Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/port_undef.inc>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_table_driven.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/metadata_lite.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>  // IWYU pragma: export
#include <google/protobuf/extension_set.h>  // IWYU pragma: export
#include <google/protobuf/unknown_field_set.h>
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_cartographer_2fmapping_2fproto_2fscan_5fmatching_2ffast_5fcorrelative_5fscan_5fmatcher_5foptions_5f3d_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_cartographer_2fmapping_2fproto_2fscan_5fmatching_2ffast_5fcorrelative_5fscan_5fmatcher_5foptions_5f3d_2eproto {
  static const ::PROTOBUF_NAMESPACE_ID::internal::ParseTableField entries[]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::AuxiliaryParseTableField aux[]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::ParseTable schema[1]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::FieldMetadata field_metadata[];
  static const ::PROTOBUF_NAMESPACE_ID::internal::SerializationTable serialization_table[];
  static const ::PROTOBUF_NAMESPACE_ID::uint32 offsets[];
};
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_cartographer_2fmapping_2fproto_2fscan_5fmatching_2ffast_5fcorrelative_5fscan_5fmatcher_5foptions_5f3d_2eproto;
namespace cartographer {
namespace mapping {
namespace scan_matching {
namespace proto {
class FastCorrelativeScanMatcherOptions3D;
struct FastCorrelativeScanMatcherOptions3DDefaultTypeInternal;
extern FastCorrelativeScanMatcherOptions3DDefaultTypeInternal _FastCorrelativeScanMatcherOptions3D_default_instance_;
}  // namespace proto
}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer
PROTOBUF_NAMESPACE_OPEN
template<> ::cartographer::mapping::scan_matching::proto::FastCorrelativeScanMatcherOptions3D* Arena::CreateMaybeMessage<::cartographer::mapping::scan_matching::proto::FastCorrelativeScanMatcherOptions3D>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace cartographer {
namespace mapping {
namespace scan_matching {
namespace proto {

// ===================================================================

class FastCorrelativeScanMatcherOptions3D final :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:cartographer.mapping.scan_matching.proto.FastCorrelativeScanMatcherOptions3D) */ {
 public:
  inline FastCorrelativeScanMatcherOptions3D() : FastCorrelativeScanMatcherOptions3D(nullptr) {}
  ~FastCorrelativeScanMatcherOptions3D() override;
  explicit constexpr FastCorrelativeScanMatcherOptions3D(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized);

  FastCorrelativeScanMatcherOptions3D(const FastCorrelativeScanMatcherOptions3D& from);
  FastCorrelativeScanMatcherOptions3D(FastCorrelativeScanMatcherOptions3D&& from) noexcept
    : FastCorrelativeScanMatcherOptions3D() {
    *this = ::std::move(from);
  }

  inline FastCorrelativeScanMatcherOptions3D& operator=(const FastCorrelativeScanMatcherOptions3D& from) {
    CopyFrom(from);
    return *this;
  }
  inline FastCorrelativeScanMatcherOptions3D& operator=(FastCorrelativeScanMatcherOptions3D&& from) noexcept {
    if (this == &from) return *this;
    if (GetOwningArena() == from.GetOwningArena()
  #ifdef PROTOBUF_FORCE_COPY_IN_MOVE
        && GetOwningArena() != nullptr
  #endif  // !PROTOBUF_FORCE_COPY_IN_MOVE
    ) {
      InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }

  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* descriptor() {
    return GetDescriptor();
  }
  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* GetDescriptor() {
    return default_instance().GetMetadata().descriptor;
  }
  static const ::PROTOBUF_NAMESPACE_ID::Reflection* GetReflection() {
    return default_instance().GetMetadata().reflection;
  }
  static const FastCorrelativeScanMatcherOptions3D& default_instance() {
    return *internal_default_instance();
  }
  static inline const FastCorrelativeScanMatcherOptions3D* internal_default_instance() {
    return reinterpret_cast<const FastCorrelativeScanMatcherOptions3D*>(
               &_FastCorrelativeScanMatcherOptions3D_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(FastCorrelativeScanMatcherOptions3D& a, FastCorrelativeScanMatcherOptions3D& b) {
    a.Swap(&b);
  }
  inline void Swap(FastCorrelativeScanMatcherOptions3D* other) {
    if (other == this) return;
    if (GetOwningArena() == other->GetOwningArena()) {
      InternalSwap(other);
    } else {
      ::PROTOBUF_NAMESPACE_ID::internal::GenericSwap(this, other);
    }
  }
  void UnsafeArenaSwap(FastCorrelativeScanMatcherOptions3D* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetOwningArena() == other->GetOwningArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline FastCorrelativeScanMatcherOptions3D* New() const final {
    return new FastCorrelativeScanMatcherOptions3D();
  }

  FastCorrelativeScanMatcherOptions3D* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<FastCorrelativeScanMatcherOptions3D>(arena);
  }
  using ::PROTOBUF_NAMESPACE_ID::Message::CopyFrom;
  void CopyFrom(const FastCorrelativeScanMatcherOptions3D& from);
  using ::PROTOBUF_NAMESPACE_ID::Message::MergeFrom;
  void MergeFrom(const FastCorrelativeScanMatcherOptions3D& from);
  private:
  static void MergeImpl(::PROTOBUF_NAMESPACE_ID::Message* to, const ::PROTOBUF_NAMESPACE_ID::Message& from);
  public:
  PROTOBUF_ATTRIBUTE_REINITIALIZES void Clear() final;
  bool IsInitialized() const final;

  size_t ByteSizeLong() const final;
  const char* _InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) final;
  ::PROTOBUF_NAMESPACE_ID::uint8* _InternalSerialize(
      ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const final;
  int GetCachedSize() const final { return _cached_size_.Get(); }

  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const final;
  void InternalSwap(FastCorrelativeScanMatcherOptions3D* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "cartographer.mapping.scan_matching.proto.FastCorrelativeScanMatcherOptions3D";
  }
  protected:
  explicit FastCorrelativeScanMatcherOptions3D(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                       bool is_message_owned = false);
  private:
  static void ArenaDtor(void* object);
  inline void RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena* arena);
  public:

  static const ClassData _class_data_;
  const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*GetClassData() const final;

  ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadata() const final;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kMinRotationalScoreFieldNumber = 4,
    kLinearXySearchWindowFieldNumber = 5,
    kBranchAndBoundDepthFieldNumber = 2,
    kFullResolutionDepthFieldNumber = 8,
    kLinearZSearchWindowFieldNumber = 6,
    kAngularSearchWindowFieldNumber = 7,
    kMinLowResolutionScoreFieldNumber = 9,
  };
  // double min_rotational_score = 4;
  void clear_min_rotational_score();
  double min_rotational_score() const;
  void set_min_rotational_score(double value);
  private:
  double _internal_min_rotational_score() const;
  void _internal_set_min_rotational_score(double value);
  public:

  // double linear_xy_search_window = 5;
  void clear_linear_xy_search_window();
  double linear_xy_search_window() const;
  void set_linear_xy_search_window(double value);
  private:
  double _internal_linear_xy_search_window() const;
  void _internal_set_linear_xy_search_window(double value);
  public:

  // int32 branch_and_bound_depth = 2;
  void clear_branch_and_bound_depth();
  ::PROTOBUF_NAMESPACE_ID::int32 branch_and_bound_depth() const;
  void set_branch_and_bound_depth(::PROTOBUF_NAMESPACE_ID::int32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::int32 _internal_branch_and_bound_depth() const;
  void _internal_set_branch_and_bound_depth(::PROTOBUF_NAMESPACE_ID::int32 value);
  public:

  // int32 full_resolution_depth = 8;
  void clear_full_resolution_depth();
  ::PROTOBUF_NAMESPACE_ID::int32 full_resolution_depth() const;
  void set_full_resolution_depth(::PROTOBUF_NAMESPACE_ID::int32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::int32 _internal_full_resolution_depth() const;
  void _internal_set_full_resolution_depth(::PROTOBUF_NAMESPACE_ID::int32 value);
  public:

  // double linear_z_search_window = 6;
  void clear_linear_z_search_window();
  double linear_z_search_window() const;
  void set_linear_z_search_window(double value);
  private:
  double _internal_linear_z_search_window() const;
  void _internal_set_linear_z_search_window(double value);
  public:

  // double angular_search_window = 7;
  void clear_angular_search_window();
  double angular_search_window() const;
  void set_angular_search_window(double value);
  private:
  double _internal_angular_search_window() const;
  void _internal_set_angular_search_window(double value);
  public:

  // double min_low_resolution_score = 9;
  void clear_min_low_resolution_score();
  double min_low_resolution_score() const;
  void set_min_low_resolution_score(double value);
  private:
  double _internal_min_low_resolution_score() const;
  void _internal_set_min_low_resolution_score(double value);
  public:

  // @@protoc_insertion_point(class_scope:cartographer.mapping.scan_matching.proto.FastCorrelativeScanMatcherOptions3D)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  double min_rotational_score_;
  double linear_xy_search_window_;
  ::PROTOBUF_NAMESPACE_ID::int32 branch_and_bound_depth_;
  ::PROTOBUF_NAMESPACE_ID::int32 full_resolution_depth_;
  double linear_z_search_window_;
  double angular_search_window_;
  double min_low_resolution_score_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  friend struct ::TableStruct_cartographer_2fmapping_2fproto_2fscan_5fmatching_2ffast_5fcorrelative_5fscan_5fmatcher_5foptions_5f3d_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// FastCorrelativeScanMatcherOptions3D

// int32 branch_and_bound_depth = 2;
inline void FastCorrelativeScanMatcherOptions3D::clear_branch_and_bound_depth() {
  branch_and_bound_depth_ = 0;
}
inline ::PROTOBUF_NAMESPACE_ID::int32 FastCorrelativeScanMatcherOptions3D::_internal_branch_and_bound_depth() const {
  return branch_and_bound_depth_;
}
inline ::PROTOBUF_NAMESPACE_ID::int32 FastCorrelativeScanMatcherOptions3D::branch_and_bound_depth() const {
  // @@protoc_insertion_point(field_get:cartographer.mapping.scan_matching.proto.FastCorrelativeScanMatcherOptions3D.branch_and_bound_depth)
  return _internal_branch_and_bound_depth();
}
inline void FastCorrelativeScanMatcherOptions3D::_internal_set_branch_and_bound_depth(::PROTOBUF_NAMESPACE_ID::int32 value) {
  
  branch_and_bound_depth_ = value;
}
inline void FastCorrelativeScanMatcherOptions3D::set_branch_and_bound_depth(::PROTOBUF_NAMESPACE_ID::int32 value) {
  _internal_set_branch_and_bound_depth(value);
  // @@protoc_insertion_point(field_set:cartographer.mapping.scan_matching.proto.FastCorrelativeScanMatcherOptions3D.branch_and_bound_depth)
}

// int32 full_resolution_depth = 8;
inline void FastCorrelativeScanMatcherOptions3D::clear_full_resolution_depth() {
  full_resolution_depth_ = 0;
}
inline ::PROTOBUF_NAMESPACE_ID::int32 FastCorrelativeScanMatcherOptions3D::_internal_full_resolution_depth() const {
  return full_resolution_depth_;
}
inline ::PROTOBUF_NAMESPACE_ID::int32 FastCorrelativeScanMatcherOptions3D::full_resolution_depth() const {
  // @@protoc_insertion_point(field_get:cartographer.mapping.scan_matching.proto.FastCorrelativeScanMatcherOptions3D.full_resolution_depth)
  return _internal_full_resolution_depth();
}
inline void FastCorrelativeScanMatcherOptions3D::_internal_set_full_resolution_depth(::PROTOBUF_NAMESPACE_ID::int32 value) {
  
  full_resolution_depth_ = value;
}
inline void FastCorrelativeScanMatcherOptions3D::set_full_resolution_depth(::PROTOBUF_NAMESPACE_ID::int32 value) {
  _internal_set_full_resolution_depth(value);
  // @@protoc_insertion_point(field_set:cartographer.mapping.scan_matching.proto.FastCorrelativeScanMatcherOptions3D.full_resolution_depth)
}

// double min_rotational_score = 4;
inline void FastCorrelativeScanMatcherOptions3D::clear_min_rotational_score() {
  min_rotational_score_ = 0;
}
inline double FastCorrelativeScanMatcherOptions3D::_internal_min_rotational_score() const {
  return min_rotational_score_;
}
inline double FastCorrelativeScanMatcherOptions3D::min_rotational_score() const {
  // @@protoc_insertion_point(field_get:cartographer.mapping.scan_matching.proto.FastCorrelativeScanMatcherOptions3D.min_rotational_score)
  return _internal_min_rotational_score();
}
inline void FastCorrelativeScanMatcherOptions3D::_internal_set_min_rotational_score(double value) {
  
  min_rotational_score_ = value;
}
inline void FastCorrelativeScanMatcherOptions3D::set_min_rotational_score(double value) {
  _internal_set_min_rotational_score(value);
  // @@protoc_insertion_point(field_set:cartographer.mapping.scan_matching.proto.FastCorrelativeScanMatcherOptions3D.min_rotational_score)
}

// double min_low_resolution_score = 9;
inline void FastCorrelativeScanMatcherOptions3D::clear_min_low_resolution_score() {
  min_low_resolution_score_ = 0;
}
inline double FastCorrelativeScanMatcherOptions3D::_internal_min_low_resolution_score() const {
  return min_low_resolution_score_;
}
inline double FastCorrelativeScanMatcherOptions3D::min_low_resolution_score() const {
  // @@protoc_insertion_point(field_get:cartographer.mapping.scan_matching.proto.FastCorrelativeScanMatcherOptions3D.min_low_resolution_score)
  return _internal_min_low_resolution_score();
}
inline void FastCorrelativeScanMatcherOptions3D::_internal_set_min_low_resolution_score(double value) {
  
  min_low_resolution_score_ = value;
}
inline void FastCorrelativeScanMatcherOptions3D::set_min_low_resolution_score(double value) {
  _internal_set_min_low_resolution_score(value);
  // @@protoc_insertion_point(field_set:cartographer.mapping.scan_matching.proto.FastCorrelativeScanMatcherOptions3D.min_low_resolution_score)
}

// double linear_xy_search_window = 5;
inline void FastCorrelativeScanMatcherOptions3D::clear_linear_xy_search_window() {
  linear_xy_search_window_ = 0;
}
inline double FastCorrelativeScanMatcherOptions3D::_internal_linear_xy_search_window() const {
  return linear_xy_search_window_;
}
inline double FastCorrelativeScanMatcherOptions3D::linear_xy_search_window() const {
  // @@protoc_insertion_point(field_get:cartographer.mapping.scan_matching.proto.FastCorrelativeScanMatcherOptions3D.linear_xy_search_window)
  return _internal_linear_xy_search_window();
}
inline void FastCorrelativeScanMatcherOptions3D::_internal_set_linear_xy_search_window(double value) {
  
  linear_xy_search_window_ = value;
}
inline void FastCorrelativeScanMatcherOptions3D::set_linear_xy_search_window(double value) {
  _internal_set_linear_xy_search_window(value);
  // @@protoc_insertion_point(field_set:cartographer.mapping.scan_matching.proto.FastCorrelativeScanMatcherOptions3D.linear_xy_search_window)
}

// double linear_z_search_window = 6;
inline void FastCorrelativeScanMatcherOptions3D::clear_linear_z_search_window() {
  linear_z_search_window_ = 0;
}
inline double FastCorrelativeScanMatcherOptions3D::_internal_linear_z_search_window() const {
  return linear_z_search_window_;
}
inline double FastCorrelativeScanMatcherOptions3D::linear_z_search_window() const {
  // @@protoc_insertion_point(field_get:cartographer.mapping.scan_matching.proto.FastCorrelativeScanMatcherOptions3D.linear_z_search_window)
  return _internal_linear_z_search_window();
}
inline void FastCorrelativeScanMatcherOptions3D::_internal_set_linear_z_search_window(double value) {
  
  linear_z_search_window_ = value;
}
inline void FastCorrelativeScanMatcherOptions3D::set_linear_z_search_window(double value) {
  _internal_set_linear_z_search_window(value);
  // @@protoc_insertion_point(field_set:cartographer.mapping.scan_matching.proto.FastCorrelativeScanMatcherOptions3D.linear_z_search_window)
}

// double angular_search_window = 7;
inline void FastCorrelativeScanMatcherOptions3D::clear_angular_search_window() {
  angular_search_window_ = 0;
}
inline double FastCorrelativeScanMatcherOptions3D::_internal_angular_search_window() const {
  return angular_search_window_;
}
inline double FastCorrelativeScanMatcherOptions3D::angular_search_window() const {
  // @@protoc_insertion_point(field_get:cartographer.mapping.scan_matching.proto.FastCorrelativeScanMatcherOptions3D.angular_search_window)
  return _internal_angular_search_window();
}
inline void FastCorrelativeScanMatcherOptions3D::_internal_set_angular_search_window(double value) {
  
  angular_search_window_ = value;
}
inline void FastCorrelativeScanMatcherOptions3D::set_angular_search_window(double value) {
  _internal_set_angular_search_window(value);
  // @@protoc_insertion_point(field_set:cartographer.mapping.scan_matching.proto.FastCorrelativeScanMatcherOptions3D.angular_search_window)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace proto
}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_cartographer_2fmapping_2fproto_2fscan_5fmatching_2ffast_5fcorrelative_5fscan_5fmatcher_5foptions_5f3d_2eproto