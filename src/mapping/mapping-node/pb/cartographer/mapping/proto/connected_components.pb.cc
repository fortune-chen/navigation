// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: cartographer/mapping/proto/connected_components.proto

#include "cartographer/mapping/proto/connected_components.pb.h"

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
constexpr ConnectedComponents_ConnectedComponent::ConnectedComponents_ConnectedComponent(
  ::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized)
  : trajectory_id_()
  , _trajectory_id_cached_byte_size_(0){}
struct ConnectedComponents_ConnectedComponentDefaultTypeInternal {
  constexpr ConnectedComponents_ConnectedComponentDefaultTypeInternal()
    : _instance(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized{}) {}
  ~ConnectedComponents_ConnectedComponentDefaultTypeInternal() {}
  union {
    ConnectedComponents_ConnectedComponent _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT ConnectedComponents_ConnectedComponentDefaultTypeInternal _ConnectedComponents_ConnectedComponent_default_instance_;
constexpr ConnectedComponents::ConnectedComponents(
  ::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized)
  : connected_component_(){}
struct ConnectedComponentsDefaultTypeInternal {
  constexpr ConnectedComponentsDefaultTypeInternal()
    : _instance(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized{}) {}
  ~ConnectedComponentsDefaultTypeInternal() {}
  union {
    ConnectedComponents _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT ConnectedComponentsDefaultTypeInternal _ConnectedComponents_default_instance_;
}  // namespace proto
}  // namespace mapping
}  // namespace cartographer
static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_cartographer_2fmapping_2fproto_2fconnected_5fcomponents_2eproto[2];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_cartographer_2fmapping_2fproto_2fconnected_5fcomponents_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_cartographer_2fmapping_2fproto_2fconnected_5fcomponents_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_cartographer_2fmapping_2fproto_2fconnected_5fcomponents_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  ~0u,  // no _has_bits_
  PROTOBUF_FIELD_OFFSET(::cartographer::mapping::proto::ConnectedComponents_ConnectedComponent, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  ~0u,  // no _inlined_string_donated_
  PROTOBUF_FIELD_OFFSET(::cartographer::mapping::proto::ConnectedComponents_ConnectedComponent, trajectory_id_),
  ~0u,  // no _has_bits_
  PROTOBUF_FIELD_OFFSET(::cartographer::mapping::proto::ConnectedComponents, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  ~0u,  // no _inlined_string_donated_
  PROTOBUF_FIELD_OFFSET(::cartographer::mapping::proto::ConnectedComponents, connected_component_),
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, -1, -1, sizeof(::cartographer::mapping::proto::ConnectedComponents_ConnectedComponent)},
  { 7, -1, -1, sizeof(::cartographer::mapping::proto::ConnectedComponents)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::cartographer::mapping::proto::_ConnectedComponents_ConnectedComponent_default_instance_),
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::cartographer::mapping::proto::_ConnectedComponents_default_instance_),
};

const char descriptor_table_protodef_cartographer_2fmapping_2fproto_2fconnected_5fcomponents_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n5cartographer/mapping/proto/connected_c"
  "omponents.proto\022\032cartographer.mapping.pr"
  "oto\"\243\001\n\023ConnectedComponents\022_\n\023connected"
  "_component\030\001 \003(\0132B.cartographer.mapping."
  "proto.ConnectedComponents.ConnectedCompo"
  "nent\032+\n\022ConnectedComponent\022\025\n\rtrajectory"
  "_id\030\001 \003(\005B\037B\035ConnectedComponentsOuterCla"
  "ssb\006proto3"
  ;
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_cartographer_2fmapping_2fproto_2fconnected_5fcomponents_2eproto_once;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_cartographer_2fmapping_2fproto_2fconnected_5fcomponents_2eproto = {
  false, false, 290, descriptor_table_protodef_cartographer_2fmapping_2fproto_2fconnected_5fcomponents_2eproto, "cartographer/mapping/proto/connected_components.proto", 
  &descriptor_table_cartographer_2fmapping_2fproto_2fconnected_5fcomponents_2eproto_once, nullptr, 0, 2,
  schemas, file_default_instances, TableStruct_cartographer_2fmapping_2fproto_2fconnected_5fcomponents_2eproto::offsets,
  file_level_metadata_cartographer_2fmapping_2fproto_2fconnected_5fcomponents_2eproto, file_level_enum_descriptors_cartographer_2fmapping_2fproto_2fconnected_5fcomponents_2eproto, file_level_service_descriptors_cartographer_2fmapping_2fproto_2fconnected_5fcomponents_2eproto,
};
PROTOBUF_ATTRIBUTE_WEAK const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable* descriptor_table_cartographer_2fmapping_2fproto_2fconnected_5fcomponents_2eproto_getter() {
  return &descriptor_table_cartographer_2fmapping_2fproto_2fconnected_5fcomponents_2eproto;
}

// Force running AddDescriptors() at dynamic initialization time.
PROTOBUF_ATTRIBUTE_INIT_PRIORITY static ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptorsRunner dynamic_init_dummy_cartographer_2fmapping_2fproto_2fconnected_5fcomponents_2eproto(&descriptor_table_cartographer_2fmapping_2fproto_2fconnected_5fcomponents_2eproto);
namespace cartographer {
namespace mapping {
namespace proto {

// ===================================================================

class ConnectedComponents_ConnectedComponent::_Internal {
 public:
};

ConnectedComponents_ConnectedComponent::ConnectedComponents_ConnectedComponent(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena, is_message_owned),
  trajectory_id_(arena) {
  SharedCtor();
  if (!is_message_owned) {
    RegisterArenaDtor(arena);
  }
  // @@protoc_insertion_point(arena_constructor:cartographer.mapping.proto.ConnectedComponents.ConnectedComponent)
}
ConnectedComponents_ConnectedComponent::ConnectedComponents_ConnectedComponent(const ConnectedComponents_ConnectedComponent& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      trajectory_id_(from.trajectory_id_) {
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  // @@protoc_insertion_point(copy_constructor:cartographer.mapping.proto.ConnectedComponents.ConnectedComponent)
}

void ConnectedComponents_ConnectedComponent::SharedCtor() {
}

ConnectedComponents_ConnectedComponent::~ConnectedComponents_ConnectedComponent() {
  // @@protoc_insertion_point(destructor:cartographer.mapping.proto.ConnectedComponents.ConnectedComponent)
  if (GetArenaForAllocation() != nullptr) return;
  SharedDtor();
  _internal_metadata_.Delete<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

inline void ConnectedComponents_ConnectedComponent::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
}

void ConnectedComponents_ConnectedComponent::ArenaDtor(void* object) {
  ConnectedComponents_ConnectedComponent* _this = reinterpret_cast< ConnectedComponents_ConnectedComponent* >(object);
  (void)_this;
}
void ConnectedComponents_ConnectedComponent::RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena*) {
}
void ConnectedComponents_ConnectedComponent::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}

void ConnectedComponents_ConnectedComponent::Clear() {
// @@protoc_insertion_point(message_clear_start:cartographer.mapping.proto.ConnectedComponents.ConnectedComponent)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  trajectory_id_.Clear();
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* ConnectedComponents_ConnectedComponent::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    switch (tag >> 3) {
      // repeated int32 trajectory_id = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 10)) {
          ptr = ::PROTOBUF_NAMESPACE_ID::internal::PackedInt32Parser(_internal_mutable_trajectory_id(), ptr, ctx);
          CHK_(ptr);
        } else if (static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 8) {
          _internal_add_trajectory_id(::PROTOBUF_NAMESPACE_ID::internal::ReadVarint64(&ptr));
          CHK_(ptr);
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

::PROTOBUF_NAMESPACE_ID::uint8* ConnectedComponents_ConnectedComponent::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:cartographer.mapping.proto.ConnectedComponents.ConnectedComponent)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // repeated int32 trajectory_id = 1;
  {
    int byte_size = _trajectory_id_cached_byte_size_.load(std::memory_order_relaxed);
    if (byte_size > 0) {
      target = stream->WriteInt32Packed(
          1, _internal_trajectory_id(), byte_size, target);
    }
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:cartographer.mapping.proto.ConnectedComponents.ConnectedComponent)
  return target;
}

size_t ConnectedComponents_ConnectedComponent::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:cartographer.mapping.proto.ConnectedComponents.ConnectedComponent)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated int32 trajectory_id = 1;
  {
    size_t data_size = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      Int32Size(this->trajectory_id_);
    if (data_size > 0) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::Int32Size(
            static_cast<::PROTOBUF_NAMESPACE_ID::int32>(data_size));
    }
    int cached_size = ::PROTOBUF_NAMESPACE_ID::internal::ToCachedSize(data_size);
    _trajectory_id_cached_byte_size_.store(cached_size,
                                    std::memory_order_relaxed);
    total_size += data_size;
  }

  return MaybeComputeUnknownFieldsSize(total_size, &_cached_size_);
}

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData ConnectedComponents_ConnectedComponent::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSizeCheck,
    ConnectedComponents_ConnectedComponent::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*ConnectedComponents_ConnectedComponent::GetClassData() const { return &_class_data_; }

void ConnectedComponents_ConnectedComponent::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message* to,
                      const ::PROTOBUF_NAMESPACE_ID::Message& from) {
  static_cast<ConnectedComponents_ConnectedComponent *>(to)->MergeFrom(
      static_cast<const ConnectedComponents_ConnectedComponent &>(from));
}


void ConnectedComponents_ConnectedComponent::MergeFrom(const ConnectedComponents_ConnectedComponent& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:cartographer.mapping.proto.ConnectedComponents.ConnectedComponent)
  GOOGLE_DCHECK_NE(&from, this);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  trajectory_id_.MergeFrom(from.trajectory_id_);
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void ConnectedComponents_ConnectedComponent::CopyFrom(const ConnectedComponents_ConnectedComponent& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:cartographer.mapping.proto.ConnectedComponents.ConnectedComponent)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool ConnectedComponents_ConnectedComponent::IsInitialized() const {
  return true;
}

void ConnectedComponents_ConnectedComponent::InternalSwap(ConnectedComponents_ConnectedComponent* other) {
  using std::swap;
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  trajectory_id_.InternalSwap(&other->trajectory_id_);
}

::PROTOBUF_NAMESPACE_ID::Metadata ConnectedComponents_ConnectedComponent::GetMetadata() const {
  return ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(
      &descriptor_table_cartographer_2fmapping_2fproto_2fconnected_5fcomponents_2eproto_getter, &descriptor_table_cartographer_2fmapping_2fproto_2fconnected_5fcomponents_2eproto_once,
      file_level_metadata_cartographer_2fmapping_2fproto_2fconnected_5fcomponents_2eproto[0]);
}

// ===================================================================

class ConnectedComponents::_Internal {
 public:
};

ConnectedComponents::ConnectedComponents(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena, is_message_owned),
  connected_component_(arena) {
  SharedCtor();
  if (!is_message_owned) {
    RegisterArenaDtor(arena);
  }
  // @@protoc_insertion_point(arena_constructor:cartographer.mapping.proto.ConnectedComponents)
}
ConnectedComponents::ConnectedComponents(const ConnectedComponents& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      connected_component_(from.connected_component_) {
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  // @@protoc_insertion_point(copy_constructor:cartographer.mapping.proto.ConnectedComponents)
}

void ConnectedComponents::SharedCtor() {
}

ConnectedComponents::~ConnectedComponents() {
  // @@protoc_insertion_point(destructor:cartographer.mapping.proto.ConnectedComponents)
  if (GetArenaForAllocation() != nullptr) return;
  SharedDtor();
  _internal_metadata_.Delete<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

inline void ConnectedComponents::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
}

void ConnectedComponents::ArenaDtor(void* object) {
  ConnectedComponents* _this = reinterpret_cast< ConnectedComponents* >(object);
  (void)_this;
}
void ConnectedComponents::RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena*) {
}
void ConnectedComponents::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}

void ConnectedComponents::Clear() {
// @@protoc_insertion_point(message_clear_start:cartographer.mapping.proto.ConnectedComponents)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  connected_component_.Clear();
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* ConnectedComponents::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    switch (tag >> 3) {
      // repeated .cartographer.mapping.proto.ConnectedComponents.ConnectedComponent connected_component = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 10)) {
          ptr -= 1;
          do {
            ptr += 1;
            ptr = ctx->ParseMessage(_internal_add_connected_component(), ptr);
            CHK_(ptr);
            if (!ctx->DataAvailable(ptr)) break;
          } while (::PROTOBUF_NAMESPACE_ID::internal::ExpectTag<10>(ptr));
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

::PROTOBUF_NAMESPACE_ID::uint8* ConnectedComponents::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:cartographer.mapping.proto.ConnectedComponents)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // repeated .cartographer.mapping.proto.ConnectedComponents.ConnectedComponent connected_component = 1;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->_internal_connected_component_size()); i < n; i++) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(1, this->_internal_connected_component(i), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:cartographer.mapping.proto.ConnectedComponents)
  return target;
}

size_t ConnectedComponents::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:cartographer.mapping.proto.ConnectedComponents)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated .cartographer.mapping.proto.ConnectedComponents.ConnectedComponent connected_component = 1;
  total_size += 1UL * this->_internal_connected_component_size();
  for (const auto& msg : this->connected_component_) {
    total_size +=
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(msg);
  }

  return MaybeComputeUnknownFieldsSize(total_size, &_cached_size_);
}

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData ConnectedComponents::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSizeCheck,
    ConnectedComponents::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*ConnectedComponents::GetClassData() const { return &_class_data_; }

void ConnectedComponents::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message* to,
                      const ::PROTOBUF_NAMESPACE_ID::Message& from) {
  static_cast<ConnectedComponents *>(to)->MergeFrom(
      static_cast<const ConnectedComponents &>(from));
}


void ConnectedComponents::MergeFrom(const ConnectedComponents& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:cartographer.mapping.proto.ConnectedComponents)
  GOOGLE_DCHECK_NE(&from, this);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  connected_component_.MergeFrom(from.connected_component_);
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void ConnectedComponents::CopyFrom(const ConnectedComponents& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:cartographer.mapping.proto.ConnectedComponents)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool ConnectedComponents::IsInitialized() const {
  return true;
}

void ConnectedComponents::InternalSwap(ConnectedComponents* other) {
  using std::swap;
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  connected_component_.InternalSwap(&other->connected_component_);
}

::PROTOBUF_NAMESPACE_ID::Metadata ConnectedComponents::GetMetadata() const {
  return ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(
      &descriptor_table_cartographer_2fmapping_2fproto_2fconnected_5fcomponents_2eproto_getter, &descriptor_table_cartographer_2fmapping_2fproto_2fconnected_5fcomponents_2eproto_once,
      file_level_metadata_cartographer_2fmapping_2fproto_2fconnected_5fcomponents_2eproto[1]);
}

// @@protoc_insertion_point(namespace_scope)
}  // namespace proto
}  // namespace mapping
}  // namespace cartographer
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::cartographer::mapping::proto::ConnectedComponents_ConnectedComponent* Arena::CreateMaybeMessage< ::cartographer::mapping::proto::ConnectedComponents_ConnectedComponent >(Arena* arena) {
  return Arena::CreateMessageInternal< ::cartographer::mapping::proto::ConnectedComponents_ConnectedComponent >(arena);
}
template<> PROTOBUF_NOINLINE ::cartographer::mapping::proto::ConnectedComponents* Arena::CreateMaybeMessage< ::cartographer::mapping::proto::ConnectedComponents >(Arena* arena) {
  return Arena::CreateMessageInternal< ::cartographer::mapping::proto::ConnectedComponents >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
