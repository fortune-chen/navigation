// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: cartographer/mapping/proto/submaps_options_3d.proto

#include "cartographer/mapping/proto/submaps_options_3d.pb.h"

#include <algorithm>

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/extension_set.h>
#include <google/protobuf/wire_format_lite.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>

PROTOBUF_PRAGMA_INIT_SEG
namespace cartographer {
namespace mapping {
namespace proto {
constexpr SubmapsOptions3D::SubmapsOptions3D(
  ::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized)
  : range_data_inserter_options_(nullptr)
  , high_resolution_(0)
  , high_resolution_max_range_(0)
  , low_resolution_(0)
  , num_range_data_(0){}
struct SubmapsOptions3DDefaultTypeInternal {
  constexpr SubmapsOptions3DDefaultTypeInternal()
    : _instance(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized{}) {}
  ~SubmapsOptions3DDefaultTypeInternal() {}
  union {
    SubmapsOptions3D _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT SubmapsOptions3DDefaultTypeInternal _SubmapsOptions3D_default_instance_;
}  // namespace proto
}  // namespace mapping
}  // namespace cartographer
static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_cartographer_2fmapping_2fproto_2fsubmaps_5foptions_5f3d_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_cartographer_2fmapping_2fproto_2fsubmaps_5foptions_5f3d_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_cartographer_2fmapping_2fproto_2fsubmaps_5foptions_5f3d_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_cartographer_2fmapping_2fproto_2fsubmaps_5foptions_5f3d_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  ~0u,  // no _has_bits_
  PROTOBUF_FIELD_OFFSET(::cartographer::mapping::proto::SubmapsOptions3D, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  ~0u,  // no _inlined_string_donated_
  PROTOBUF_FIELD_OFFSET(::cartographer::mapping::proto::SubmapsOptions3D, high_resolution_),
  PROTOBUF_FIELD_OFFSET(::cartographer::mapping::proto::SubmapsOptions3D, high_resolution_max_range_),
  PROTOBUF_FIELD_OFFSET(::cartographer::mapping::proto::SubmapsOptions3D, low_resolution_),
  PROTOBUF_FIELD_OFFSET(::cartographer::mapping::proto::SubmapsOptions3D, num_range_data_),
  PROTOBUF_FIELD_OFFSET(::cartographer::mapping::proto::SubmapsOptions3D, range_data_inserter_options_),
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, -1, -1, sizeof(::cartographer::mapping::proto::SubmapsOptions3D)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::cartographer::mapping::proto::_SubmapsOptions3D_default_instance_),
};

const char descriptor_table_protodef_cartographer_2fmapping_2fproto_2fsubmaps_5foptions_5f3d_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n3cartographer/mapping/proto/submaps_opt"
  "ions_3d.proto\022\032cartographer.mapping.prot"
  "o\032\?cartographer/mapping/proto/range_data"
  "_inserter_options_3d.proto\"\333\001\n\020SubmapsOp"
  "tions3D\022\027\n\017high_resolution\030\001 \001(\001\022!\n\031high"
  "_resolution_max_range\030\004 \001(\001\022\026\n\016low_resol"
  "ution\030\005 \001(\001\022\026\n\016num_range_data\030\002 \001(\005\022[\n\033r"
  "ange_data_inserter_options\030\003 \001(\01326.carto"
  "grapher.mapping.proto.RangeDataInserterO"
  "ptions3Db\006proto3"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_cartographer_2fmapping_2fproto_2fsubmaps_5foptions_5f3d_2eproto_deps[1] = {
  &::descriptor_table_cartographer_2fmapping_2fproto_2frange_5fdata_5finserter_5foptions_5f3d_2eproto,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_cartographer_2fmapping_2fproto_2fsubmaps_5foptions_5f3d_2eproto_once;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_cartographer_2fmapping_2fproto_2fsubmaps_5foptions_5f3d_2eproto = {
  false, false, 376, descriptor_table_protodef_cartographer_2fmapping_2fproto_2fsubmaps_5foptions_5f3d_2eproto, "cartographer/mapping/proto/submaps_options_3d.proto", 
  &descriptor_table_cartographer_2fmapping_2fproto_2fsubmaps_5foptions_5f3d_2eproto_once, descriptor_table_cartographer_2fmapping_2fproto_2fsubmaps_5foptions_5f3d_2eproto_deps, 1, 1,
  schemas, file_default_instances, TableStruct_cartographer_2fmapping_2fproto_2fsubmaps_5foptions_5f3d_2eproto::offsets,
  file_level_metadata_cartographer_2fmapping_2fproto_2fsubmaps_5foptions_5f3d_2eproto, file_level_enum_descriptors_cartographer_2fmapping_2fproto_2fsubmaps_5foptions_5f3d_2eproto, file_level_service_descriptors_cartographer_2fmapping_2fproto_2fsubmaps_5foptions_5f3d_2eproto,
};
PROTOBUF_ATTRIBUTE_WEAK const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable* descriptor_table_cartographer_2fmapping_2fproto_2fsubmaps_5foptions_5f3d_2eproto_getter() {
  return &descriptor_table_cartographer_2fmapping_2fproto_2fsubmaps_5foptions_5f3d_2eproto;
}

// Force running AddDescriptors() at dynamic initialization time.
PROTOBUF_ATTRIBUTE_INIT_PRIORITY static ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptorsRunner dynamic_init_dummy_cartographer_2fmapping_2fproto_2fsubmaps_5foptions_5f3d_2eproto(&descriptor_table_cartographer_2fmapping_2fproto_2fsubmaps_5foptions_5f3d_2eproto);
namespace cartographer {
namespace mapping {
namespace proto {

// ===================================================================

class SubmapsOptions3D::_Internal {
 public:
  static const ::cartographer::mapping::proto::RangeDataInserterOptions3D& range_data_inserter_options(const SubmapsOptions3D* msg);
};

const ::cartographer::mapping::proto::RangeDataInserterOptions3D&
SubmapsOptions3D::_Internal::range_data_inserter_options(const SubmapsOptions3D* msg) {
  return *msg->range_data_inserter_options_;
}
void SubmapsOptions3D::clear_range_data_inserter_options() {
  if (GetArenaForAllocation() == nullptr && range_data_inserter_options_ != nullptr) {
    delete range_data_inserter_options_;
  }
  range_data_inserter_options_ = nullptr;
}
SubmapsOptions3D::SubmapsOptions3D(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena, is_message_owned) {
  SharedCtor();
  if (!is_message_owned) {
    RegisterArenaDtor(arena);
  }
  // @@protoc_insertion_point(arena_constructor:cartographer.mapping.proto.SubmapsOptions3D)
}
SubmapsOptions3D::SubmapsOptions3D(const SubmapsOptions3D& from)
  : ::PROTOBUF_NAMESPACE_ID::Message() {
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  if (from._internal_has_range_data_inserter_options()) {
    range_data_inserter_options_ = new ::cartographer::mapping::proto::RangeDataInserterOptions3D(*from.range_data_inserter_options_);
  } else {
    range_data_inserter_options_ = nullptr;
  }
  ::memcpy(&high_resolution_, &from.high_resolution_,
    static_cast<size_t>(reinterpret_cast<char*>(&num_range_data_) -
    reinterpret_cast<char*>(&high_resolution_)) + sizeof(num_range_data_));
  // @@protoc_insertion_point(copy_constructor:cartographer.mapping.proto.SubmapsOptions3D)
}

void SubmapsOptions3D::SharedCtor() {
::memset(reinterpret_cast<char*>(this) + static_cast<size_t>(
    reinterpret_cast<char*>(&range_data_inserter_options_) - reinterpret_cast<char*>(this)),
    0, static_cast<size_t>(reinterpret_cast<char*>(&num_range_data_) -
    reinterpret_cast<char*>(&range_data_inserter_options_)) + sizeof(num_range_data_));
}

SubmapsOptions3D::~SubmapsOptions3D() {
  // @@protoc_insertion_point(destructor:cartographer.mapping.proto.SubmapsOptions3D)
  if (GetArenaForAllocation() != nullptr) return;
  SharedDtor();
  _internal_metadata_.Delete<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

inline void SubmapsOptions3D::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
  if (this != internal_default_instance()) delete range_data_inserter_options_;
}

void SubmapsOptions3D::ArenaDtor(void* object) {
  SubmapsOptions3D* _this = reinterpret_cast< SubmapsOptions3D* >(object);
  (void)_this;
}
void SubmapsOptions3D::RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena*) {
}
void SubmapsOptions3D::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}

void SubmapsOptions3D::Clear() {
// @@protoc_insertion_point(message_clear_start:cartographer.mapping.proto.SubmapsOptions3D)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  if (GetArenaForAllocation() == nullptr && range_data_inserter_options_ != nullptr) {
    delete range_data_inserter_options_;
  }
  range_data_inserter_options_ = nullptr;
  ::memset(&high_resolution_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&num_range_data_) -
      reinterpret_cast<char*>(&high_resolution_)) + sizeof(num_range_data_));
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* SubmapsOptions3D::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    switch (tag >> 3) {
      // double high_resolution = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 9)) {
          high_resolution_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr);
          ptr += sizeof(double);
        } else
          goto handle_unusual;
        continue;
      // int32 num_range_data = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 16)) {
          num_range_data_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint64(&ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // .cartographer.mapping.proto.RangeDataInserterOptions3D range_data_inserter_options = 3;
      case 3:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 26)) {
          ptr = ctx->ParseMessage(_internal_mutable_range_data_inserter_options(), ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // double high_resolution_max_range = 4;
      case 4:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 33)) {
          high_resolution_max_range_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr);
          ptr += sizeof(double);
        } else
          goto handle_unusual;
        continue;
      // double low_resolution = 5;
      case 5:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 41)) {
          low_resolution_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr);
          ptr += sizeof(double);
        } else
          goto handle_unusual;
        continue;
      default:
        goto handle_unusual;
    }  // switch
  handle_unusual:
    if ((tag == 0) || ((tag & 7) == 4)) {
      CHK_(ptr);
      ctx->SetLastTag(tag);
      goto message_done;
    }
    ptr = UnknownFieldParse(
        tag,
        _internal_metadata_.mutable_unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(),
        ptr, ctx);
    CHK_(ptr != nullptr);
  }  // while
message_done:
  return ptr;
failure:
  ptr = nullptr;
  goto message_done;
#undef CHK_
}

::PROTOBUF_NAMESPACE_ID::uint8* SubmapsOptions3D::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:cartographer.mapping.proto.SubmapsOptions3D)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // double high_resolution = 1;
  if (!(this->_internal_high_resolution() <= 0 && this->_internal_high_resolution() >= 0)) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteDoubleToArray(1, this->_internal_high_resolution(), target);
  }

  // int32 num_range_data = 2;
  if (this->_internal_num_range_data() != 0) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteInt32ToArray(2, this->_internal_num_range_data(), target);
  }

  // .cartographer.mapping.proto.RangeDataInserterOptions3D range_data_inserter_options = 3;
  if (this->_internal_has_range_data_inserter_options()) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        3, _Internal::range_data_inserter_options(this), target, stream);
  }

  // double high_resolution_max_range = 4;
  if (!(this->_internal_high_resolution_max_range() <= 0 && this->_internal_high_resolution_max_range() >= 0)) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteDoubleToArray(4, this->_internal_high_resolution_max_range(), target);
  }

  // double low_resolution = 5;
  if (!(this->_internal_low_resolution() <= 0 && this->_internal_low_resolution() >= 0)) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteDoubleToArray(5, this->_internal_low_resolution(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:cartographer.mapping.proto.SubmapsOptions3D)
  return target;
}

size_t SubmapsOptions3D::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:cartographer.mapping.proto.SubmapsOptions3D)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // .cartographer.mapping.proto.RangeDataInserterOptions3D range_data_inserter_options = 3;
  if (this->_internal_has_range_data_inserter_options()) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
        *range_data_inserter_options_);
  }

  // double high_resolution = 1;
  if (!(this->_internal_high_resolution() <= 0 && this->_internal_high_resolution() >= 0)) {
    total_size += 1 + 8;
  }

  // double high_resolution_max_range = 4;
  if (!(this->_internal_high_resolution_max_range() <= 0 && this->_internal_high_resolution_max_range() >= 0)) {
    total_size += 1 + 8;
  }

  // double low_resolution = 5;
  if (!(this->_internal_low_resolution() <= 0 && this->_internal_low_resolution() >= 0)) {
    total_size += 1 + 8;
  }

  // int32 num_range_data = 2;
  if (this->_internal_num_range_data() != 0) {
    total_size += ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::Int32SizePlusOne(this->_internal_num_range_data());
  }

  return MaybeComputeUnknownFieldsSize(total_size, &_cached_size_);
}

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData SubmapsOptions3D::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSizeCheck,
    SubmapsOptions3D::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*SubmapsOptions3D::GetClassData() const { return &_class_data_; }

void SubmapsOptions3D::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message* to,
                      const ::PROTOBUF_NAMESPACE_ID::Message& from) {
  static_cast<SubmapsOptions3D *>(to)->MergeFrom(
      static_cast<const SubmapsOptions3D &>(from));
}


void SubmapsOptions3D::MergeFrom(const SubmapsOptions3D& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:cartographer.mapping.proto.SubmapsOptions3D)
  GOOGLE_DCHECK_NE(&from, this);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  if (from._internal_has_range_data_inserter_options()) {
    _internal_mutable_range_data_inserter_options()->::cartographer::mapping::proto::RangeDataInserterOptions3D::MergeFrom(from._internal_range_data_inserter_options());
  }
  if (!(from._internal_high_resolution() <= 0 && from._internal_high_resolution() >= 0)) {
    _internal_set_high_resolution(from._internal_high_resolution());
  }
  if (!(from._internal_high_resolution_max_range() <= 0 && from._internal_high_resolution_max_range() >= 0)) {
    _internal_set_high_resolution_max_range(from._internal_high_resolution_max_range());
  }
  if (!(from._internal_low_resolution() <= 0 && from._internal_low_resolution() >= 0)) {
    _internal_set_low_resolution(from._internal_low_resolution());
  }
  if (from._internal_num_range_data() != 0) {
    _internal_set_num_range_data(from._internal_num_range_data());
  }
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void SubmapsOptions3D::CopyFrom(const SubmapsOptions3D& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:cartographer.mapping.proto.SubmapsOptions3D)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool SubmapsOptions3D::IsInitialized() const {
  return true;
}

void SubmapsOptions3D::InternalSwap(SubmapsOptions3D* other) {
  using std::swap;
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::internal::memswap<
      PROTOBUF_FIELD_OFFSET(SubmapsOptions3D, num_range_data_)
      + sizeof(SubmapsOptions3D::num_range_data_)
      - PROTOBUF_FIELD_OFFSET(SubmapsOptions3D, range_data_inserter_options_)>(
          reinterpret_cast<char*>(&range_data_inserter_options_),
          reinterpret_cast<char*>(&other->range_data_inserter_options_));
}

::PROTOBUF_NAMESPACE_ID::Metadata SubmapsOptions3D::GetMetadata() const {
  return ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(
      &descriptor_table_cartographer_2fmapping_2fproto_2fsubmaps_5foptions_5f3d_2eproto_getter, &descriptor_table_cartographer_2fmapping_2fproto_2fsubmaps_5foptions_5f3d_2eproto_once,
      file_level_metadata_cartographer_2fmapping_2fproto_2fsubmaps_5foptions_5f3d_2eproto[0]);
}

// @@protoc_insertion_point(namespace_scope)
}  // namespace proto
}  // namespace mapping
}  // namespace cartographer
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::cartographer::mapping::proto::SubmapsOptions3D* Arena::CreateMaybeMessage< ::cartographer::mapping::proto::SubmapsOptions3D >(Arena* arena) {
  return Arena::CreateMessageInternal< ::cartographer::mapping::proto::SubmapsOptions3D >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>