// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: gps.proto

#define INTERNAL_SUPPRESS_PROTOBUF_FIELD_DEPRECATION
#include "gps.pb.h"

#include <algorithm>

#include <google/protobuf/stubs/common.h>
#include <google/protobuf/stubs/once.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/wire_format_lite_inl.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// @@protoc_insertion_point(includes)

namespace gps {

namespace {

const ::google::protobuf::Descriptor* Sample_descriptor_ = NULL;
const ::google::protobuf::internal::GeneratedMessageReflection*
  Sample_reflection_ = NULL;
const ::google::protobuf::EnumDescriptor* SampleType_descriptor_ = NULL;
const ::google::protobuf::EnumDescriptor* ActuatorType_descriptor_ = NULL;
const ::google::protobuf::EnumDescriptor* PositionControlMode_descriptor_ = NULL;
const ::google::protobuf::EnumDescriptor* ControllerType_descriptor_ = NULL;

}  // namespace


void protobuf_AssignDesc_gps_2eproto() {
  protobuf_AddDesc_gps_2eproto();
  const ::google::protobuf::FileDescriptor* file =
    ::google::protobuf::DescriptorPool::generated_pool()->FindFileByName(
      "gps.proto");
  GOOGLE_CHECK(file != NULL);
  Sample_descriptor_ = file->message_type(0);
  static const int Sample_offsets_[8] = {
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Sample, t_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Sample, dx_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Sample, du_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Sample, do__),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Sample, x_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Sample, u_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Sample, obs_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Sample, meta_),
  };
  Sample_reflection_ =
    new ::google::protobuf::internal::GeneratedMessageReflection(
      Sample_descriptor_,
      Sample::default_instance_,
      Sample_offsets_,
      GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Sample, _has_bits_[0]),
      GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Sample, _unknown_fields_),
      -1,
      ::google::protobuf::DescriptorPool::generated_pool(),
      ::google::protobuf::MessageFactory::generated_factory(),
      sizeof(Sample));
  SampleType_descriptor_ = file->enum_type(0);
  ActuatorType_descriptor_ = file->enum_type(1);
  PositionControlMode_descriptor_ = file->enum_type(2);
  ControllerType_descriptor_ = file->enum_type(3);
}

namespace {

GOOGLE_PROTOBUF_DECLARE_ONCE(protobuf_AssignDescriptors_once_);
inline void protobuf_AssignDescriptorsOnce() {
  ::google::protobuf::GoogleOnceInit(&protobuf_AssignDescriptors_once_,
                 &protobuf_AssignDesc_gps_2eproto);
}

void protobuf_RegisterTypes(const ::std::string&) {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedMessage(
    Sample_descriptor_, &Sample::default_instance());
}

}  // namespace

void protobuf_ShutdownFile_gps_2eproto() {
  delete Sample::default_instance_;
  delete Sample_reflection_;
}

void protobuf_AddDesc_gps_2eproto() {
  static bool already_here = false;
  if (already_here) return;
  already_here = true;
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
    "\n\tgps.proto\022\003gps\"}\n\006Sample\022\016\n\001T\030\001 \001(\r:\0031"
    "00\022\n\n\002dX\030\002 \001(\r\022\n\n\002dU\030\003 \001(\r\022\n\n\002dO\030\004 \001(\r\022\r"
    "\n\001X\030\005 \003(\002B\002\020\001\022\r\n\001U\030\006 \003(\002B\002\020\001\022\017\n\003obs\030\007 \003("
    "\002B\002\020\001\022\020\n\004meta\030\010 \003(\002B\002\020\001*\273\004\n\nSampleType\022\n"
    "\n\006ACTION\020\000\022\020\n\014JOINT_ANGLES\020\001\022\024\n\020JOINT_VE"
    "LOCITIES\020\002\022\027\n\023END_EFFECTOR_POINTS\020\003\022!\n\035E"
    "ND_EFFECTOR_POINT_VELOCITIES\020\004\022 \n\034END_EF"
    "FECTOR_POINT_JACOBIANS\020\005\022$\n END_EFFECTOR"
    "_POINT_ROT_JACOBIANS\020\006\022\032\n\026END_EFFECTOR_P"
    "OSITIONS\020\007\022\032\n\026END_EFFECTOR_ROTATIONS\020\010\022\032"
    "\n\026END_EFFECTOR_JACOBIANS\020\t\022\031\n\025END_EFFECT"
    "OR_HESSIANS\020\n\022\r\n\tRGB_IMAGE\020\013\022\017\n\013DEPTH_IM"
    "AGE\020\014\022\022\n\016RGB_IMAGE_SIZE\020\r\022\021\n\rCONTEXT_IMA"
    "GE\020\016\022\026\n\022CONTEXT_IMAGE_SIZE\020\017\022\016\n\nIMAGE_FE"
    "AT\020\020\022!\n\035END_EFFECTOR_POINTS_NO_TARGET\020\021\022"
    "+\n\'END_EFFECTOR_POINT_VELOCITIES_NO_TARG"
    "ET\020\022\022\025\n\021TOOL_JOINT_ANGLES\020\023\022\032\n\026TOOL_JOIN"
    "T_VELOCITYIES\020\024\022\024\n\020TOTAL_DATA_TYPES\020\025*J\n"
    "\014ActuatorType\022\r\n\tTRIAL_ARM\020\000\022\021\n\rAUXILIAR"
    "Y_ARM\020\001\022\030\n\024TOTAL_ACTUATOR_TYPES\020\002*_\n\023Pos"
    "itionControlMode\022\016\n\nNO_CONTROL\020\000\022\017\n\013JOIN"
    "T_SPACE\020\001\022\016\n\nTASK_SPACE\020\002\022\027\n\023TOTAL_CONTR"
    "OL_MODES\020\003*o\n\016ControllerType\022\030\n\024LIN_GAUS"
    "S_CONTROLLER\020\000\022\024\n\020CAFFE_CONTROLLER\020\001\022\021\n\r"
    "TF_CONTROLLER\020\002\022\032\n\026TOTAL_CONTROLLER_TYPE"
    "S\020\003", 1003);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "gps.proto", &protobuf_RegisterTypes);
  Sample::default_instance_ = new Sample();
  Sample::default_instance_->InitAsDefaultInstance();
  ::google::protobuf::internal::OnShutdown(&protobuf_ShutdownFile_gps_2eproto);
}

// Force AddDescriptors() to be called at static initialization time.
struct StaticDescriptorInitializer_gps_2eproto {
  StaticDescriptorInitializer_gps_2eproto() {
    protobuf_AddDesc_gps_2eproto();
  }
} static_descriptor_initializer_gps_2eproto_;
const ::google::protobuf::EnumDescriptor* SampleType_descriptor() {
  protobuf_AssignDescriptorsOnce();
  return SampleType_descriptor_;
}
bool SampleType_IsValid(int value) {
  switch(value) {
    case 0:
    case 1:
    case 2:
    case 3:
    case 4:
    case 5:
    case 6:
    case 7:
    case 8:
    case 9:
    case 10:
    case 11:
    case 12:
    case 13:
    case 14:
    case 15:
    case 16:
    case 17:
    case 18:
    case 19:
    case 20:
    case 21:
      return true;
    default:
      return false;
  }
}

const ::google::protobuf::EnumDescriptor* ActuatorType_descriptor() {
  protobuf_AssignDescriptorsOnce();
  return ActuatorType_descriptor_;
}
bool ActuatorType_IsValid(int value) {
  switch(value) {
    case 0:
    case 1:
    case 2:
      return true;
    default:
      return false;
  }
}

const ::google::protobuf::EnumDescriptor* PositionControlMode_descriptor() {
  protobuf_AssignDescriptorsOnce();
  return PositionControlMode_descriptor_;
}
bool PositionControlMode_IsValid(int value) {
  switch(value) {
    case 0:
    case 1:
    case 2:
    case 3:
      return true;
    default:
      return false;
  }
}

const ::google::protobuf::EnumDescriptor* ControllerType_descriptor() {
  protobuf_AssignDescriptorsOnce();
  return ControllerType_descriptor_;
}
bool ControllerType_IsValid(int value) {
  switch(value) {
    case 0:
    case 1:
    case 2:
    case 3:
      return true;
    default:
      return false;
  }
}


// ===================================================================

#ifndef _MSC_VER
const int Sample::kTFieldNumber;
const int Sample::kDXFieldNumber;
const int Sample::kDUFieldNumber;
const int Sample::kDOFieldNumber;
const int Sample::kXFieldNumber;
const int Sample::kUFieldNumber;
const int Sample::kObsFieldNumber;
const int Sample::kMetaFieldNumber;
#endif  // !_MSC_VER

Sample::Sample()
  : ::google::protobuf::Message() {
  SharedCtor();
}

void Sample::InitAsDefaultInstance() {
}

Sample::Sample(const Sample& from)
  : ::google::protobuf::Message() {
  SharedCtor();
  MergeFrom(from);
}

void Sample::SharedCtor() {
  _cached_size_ = 0;
  t_ = 100u;
  dx_ = 0u;
  du_ = 0u;
  do__ = 0u;
  ::memset(_has_bits_, 0, sizeof(_has_bits_));
}

Sample::~Sample() {
  SharedDtor();
}

void Sample::SharedDtor() {
  if (this != default_instance_) {
  }
}

void Sample::SetCachedSize(int size) const {
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
}
const ::google::protobuf::Descriptor* Sample::descriptor() {
  protobuf_AssignDescriptorsOnce();
  return Sample_descriptor_;
}

const Sample& Sample::default_instance() {
  if (default_instance_ == NULL) protobuf_AddDesc_gps_2eproto();
  return *default_instance_;
}

Sample* Sample::default_instance_ = NULL;

Sample* Sample::New() const {
  return new Sample;
}

void Sample::Clear() {
  if (_has_bits_[0 / 32] & (0xffu << (0 % 32))) {
    t_ = 100u;
    dx_ = 0u;
    du_ = 0u;
    do__ = 0u;
  }
  x_.Clear();
  u_.Clear();
  obs_.Clear();
  meta_.Clear();
  ::memset(_has_bits_, 0, sizeof(_has_bits_));
  mutable_unknown_fields()->Clear();
}

bool Sample::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!(EXPRESSION)) return false
  ::google::protobuf::uint32 tag;
  while ((tag = input->ReadTag()) != 0) {
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // optional uint32 T = 1 [default = 100];
      case 1: {
        if (::google::protobuf::internal::WireFormatLite::GetTagWireType(tag) ==
            ::google::protobuf::internal::WireFormatLite::WIRETYPE_VARINT) {
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   ::google::protobuf::uint32, ::google::protobuf::internal::WireFormatLite::TYPE_UINT32>(
                 input, &t_)));
          set_has_t();
        } else {
          goto handle_uninterpreted;
        }
        if (input->ExpectTag(16)) goto parse_dX;
        break;
      }

      // optional uint32 dX = 2;
      case 2: {
        if (::google::protobuf::internal::WireFormatLite::GetTagWireType(tag) ==
            ::google::protobuf::internal::WireFormatLite::WIRETYPE_VARINT) {
         parse_dX:
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   ::google::protobuf::uint32, ::google::protobuf::internal::WireFormatLite::TYPE_UINT32>(
                 input, &dx_)));
          set_has_dx();
        } else {
          goto handle_uninterpreted;
        }
        if (input->ExpectTag(24)) goto parse_dU;
        break;
      }

      // optional uint32 dU = 3;
      case 3: {
        if (::google::protobuf::internal::WireFormatLite::GetTagWireType(tag) ==
            ::google::protobuf::internal::WireFormatLite::WIRETYPE_VARINT) {
         parse_dU:
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   ::google::protobuf::uint32, ::google::protobuf::internal::WireFormatLite::TYPE_UINT32>(
                 input, &du_)));
          set_has_du();
        } else {
          goto handle_uninterpreted;
        }
        if (input->ExpectTag(32)) goto parse_dO;
        break;
      }

      // optional uint32 dO = 4;
      case 4: {
        if (::google::protobuf::internal::WireFormatLite::GetTagWireType(tag) ==
            ::google::protobuf::internal::WireFormatLite::WIRETYPE_VARINT) {
         parse_dO:
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   ::google::protobuf::uint32, ::google::protobuf::internal::WireFormatLite::TYPE_UINT32>(
                 input, &do__)));
          set_has_do_();
        } else {
          goto handle_uninterpreted;
        }
        if (input->ExpectTag(42)) goto parse_X;
        break;
      }

      // repeated float X = 5 [packed = true];
      case 5: {
        if (::google::protobuf::internal::WireFormatLite::GetTagWireType(tag) ==
            ::google::protobuf::internal::WireFormatLite::WIRETYPE_LENGTH_DELIMITED) {
         parse_X:
          DO_((::google::protobuf::internal::WireFormatLite::ReadPackedPrimitive<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 input, this->mutable_x())));
        } else if (::google::protobuf::internal::WireFormatLite::GetTagWireType(tag)
                   == ::google::protobuf::internal::WireFormatLite::
                      WIRETYPE_FIXED32) {
          DO_((::google::protobuf::internal::WireFormatLite::ReadRepeatedPrimitiveNoInline<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 1, 42, input, this->mutable_x())));
        } else {
          goto handle_uninterpreted;
        }
        if (input->ExpectTag(50)) goto parse_U;
        break;
      }

      // repeated float U = 6 [packed = true];
      case 6: {
        if (::google::protobuf::internal::WireFormatLite::GetTagWireType(tag) ==
            ::google::protobuf::internal::WireFormatLite::WIRETYPE_LENGTH_DELIMITED) {
         parse_U:
          DO_((::google::protobuf::internal::WireFormatLite::ReadPackedPrimitive<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 input, this->mutable_u())));
        } else if (::google::protobuf::internal::WireFormatLite::GetTagWireType(tag)
                   == ::google::protobuf::internal::WireFormatLite::
                      WIRETYPE_FIXED32) {
          DO_((::google::protobuf::internal::WireFormatLite::ReadRepeatedPrimitiveNoInline<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 1, 50, input, this->mutable_u())));
        } else {
          goto handle_uninterpreted;
        }
        if (input->ExpectTag(58)) goto parse_obs;
        break;
      }

      // repeated float obs = 7 [packed = true];
      case 7: {
        if (::google::protobuf::internal::WireFormatLite::GetTagWireType(tag) ==
            ::google::protobuf::internal::WireFormatLite::WIRETYPE_LENGTH_DELIMITED) {
         parse_obs:
          DO_((::google::protobuf::internal::WireFormatLite::ReadPackedPrimitive<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 input, this->mutable_obs())));
        } else if (::google::protobuf::internal::WireFormatLite::GetTagWireType(tag)
                   == ::google::protobuf::internal::WireFormatLite::
                      WIRETYPE_FIXED32) {
          DO_((::google::protobuf::internal::WireFormatLite::ReadRepeatedPrimitiveNoInline<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 1, 58, input, this->mutable_obs())));
        } else {
          goto handle_uninterpreted;
        }
        if (input->ExpectTag(66)) goto parse_meta;
        break;
      }

      // repeated float meta = 8 [packed = true];
      case 8: {
        if (::google::protobuf::internal::WireFormatLite::GetTagWireType(tag) ==
            ::google::protobuf::internal::WireFormatLite::WIRETYPE_LENGTH_DELIMITED) {
         parse_meta:
          DO_((::google::protobuf::internal::WireFormatLite::ReadPackedPrimitive<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 input, this->mutable_meta())));
        } else if (::google::protobuf::internal::WireFormatLite::GetTagWireType(tag)
                   == ::google::protobuf::internal::WireFormatLite::
                      WIRETYPE_FIXED32) {
          DO_((::google::protobuf::internal::WireFormatLite::ReadRepeatedPrimitiveNoInline<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 1, 66, input, this->mutable_meta())));
        } else {
          goto handle_uninterpreted;
        }
        if (input->ExpectAtEnd()) return true;
        break;
      }

      default: {
      handle_uninterpreted:
        if (::google::protobuf::internal::WireFormatLite::GetTagWireType(tag) ==
            ::google::protobuf::internal::WireFormatLite::WIRETYPE_END_GROUP) {
          return true;
        }
        DO_(::google::protobuf::internal::WireFormat::SkipField(
              input, tag, mutable_unknown_fields()));
        break;
      }
    }
  }
  return true;
#undef DO_
}

void Sample::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // optional uint32 T = 1 [default = 100];
  if (has_t()) {
    ::google::protobuf::internal::WireFormatLite::WriteUInt32(1, this->t(), output);
  }

  // optional uint32 dX = 2;
  if (has_dx()) {
    ::google::protobuf::internal::WireFormatLite::WriteUInt32(2, this->dx(), output);
  }

  // optional uint32 dU = 3;
  if (has_du()) {
    ::google::protobuf::internal::WireFormatLite::WriteUInt32(3, this->du(), output);
  }

  // optional uint32 dO = 4;
  if (has_do_()) {
    ::google::protobuf::internal::WireFormatLite::WriteUInt32(4, this->do_(), output);
  }

  // repeated float X = 5 [packed = true];
  if (this->x_size() > 0) {
    ::google::protobuf::internal::WireFormatLite::WriteTag(5, ::google::protobuf::internal::WireFormatLite::WIRETYPE_LENGTH_DELIMITED, output);
    output->WriteVarint32(_x_cached_byte_size_);
  }
  for (int i = 0; i < this->x_size(); i++) {
    ::google::protobuf::internal::WireFormatLite::WriteFloatNoTag(
      this->x(i), output);
  }

  // repeated float U = 6 [packed = true];
  if (this->u_size() > 0) {
    ::google::protobuf::internal::WireFormatLite::WriteTag(6, ::google::protobuf::internal::WireFormatLite::WIRETYPE_LENGTH_DELIMITED, output);
    output->WriteVarint32(_u_cached_byte_size_);
  }
  for (int i = 0; i < this->u_size(); i++) {
    ::google::protobuf::internal::WireFormatLite::WriteFloatNoTag(
      this->u(i), output);
  }

  // repeated float obs = 7 [packed = true];
  if (this->obs_size() > 0) {
    ::google::protobuf::internal::WireFormatLite::WriteTag(7, ::google::protobuf::internal::WireFormatLite::WIRETYPE_LENGTH_DELIMITED, output);
    output->WriteVarint32(_obs_cached_byte_size_);
  }
  for (int i = 0; i < this->obs_size(); i++) {
    ::google::protobuf::internal::WireFormatLite::WriteFloatNoTag(
      this->obs(i), output);
  }

  // repeated float meta = 8 [packed = true];
  if (this->meta_size() > 0) {
    ::google::protobuf::internal::WireFormatLite::WriteTag(8, ::google::protobuf::internal::WireFormatLite::WIRETYPE_LENGTH_DELIMITED, output);
    output->WriteVarint32(_meta_cached_byte_size_);
  }
  for (int i = 0; i < this->meta_size(); i++) {
    ::google::protobuf::internal::WireFormatLite::WriteFloatNoTag(
      this->meta(i), output);
  }

  if (!unknown_fields().empty()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        unknown_fields(), output);
  }
}

::google::protobuf::uint8* Sample::SerializeWithCachedSizesToArray(
    ::google::protobuf::uint8* target) const {
  // optional uint32 T = 1 [default = 100];
  if (has_t()) {
    target = ::google::protobuf::internal::WireFormatLite::WriteUInt32ToArray(1, this->t(), target);
  }

  // optional uint32 dX = 2;
  if (has_dx()) {
    target = ::google::protobuf::internal::WireFormatLite::WriteUInt32ToArray(2, this->dx(), target);
  }

  // optional uint32 dU = 3;
  if (has_du()) {
    target = ::google::protobuf::internal::WireFormatLite::WriteUInt32ToArray(3, this->du(), target);
  }

  // optional uint32 dO = 4;
  if (has_do_()) {
    target = ::google::protobuf::internal::WireFormatLite::WriteUInt32ToArray(4, this->do_(), target);
  }

  // repeated float X = 5 [packed = true];
  if (this->x_size() > 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteTagToArray(
      5,
      ::google::protobuf::internal::WireFormatLite::WIRETYPE_LENGTH_DELIMITED,
      target);
    target = ::google::protobuf::io::CodedOutputStream::WriteVarint32ToArray(
      _x_cached_byte_size_, target);
  }
  for (int i = 0; i < this->x_size(); i++) {
    target = ::google::protobuf::internal::WireFormatLite::
      WriteFloatNoTagToArray(this->x(i), target);
  }

  // repeated float U = 6 [packed = true];
  if (this->u_size() > 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteTagToArray(
      6,
      ::google::protobuf::internal::WireFormatLite::WIRETYPE_LENGTH_DELIMITED,
      target);
    target = ::google::protobuf::io::CodedOutputStream::WriteVarint32ToArray(
      _u_cached_byte_size_, target);
  }
  for (int i = 0; i < this->u_size(); i++) {
    target = ::google::protobuf::internal::WireFormatLite::
      WriteFloatNoTagToArray(this->u(i), target);
  }

  // repeated float obs = 7 [packed = true];
  if (this->obs_size() > 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteTagToArray(
      7,
      ::google::protobuf::internal::WireFormatLite::WIRETYPE_LENGTH_DELIMITED,
      target);
    target = ::google::protobuf::io::CodedOutputStream::WriteVarint32ToArray(
      _obs_cached_byte_size_, target);
  }
  for (int i = 0; i < this->obs_size(); i++) {
    target = ::google::protobuf::internal::WireFormatLite::
      WriteFloatNoTagToArray(this->obs(i), target);
  }

  // repeated float meta = 8 [packed = true];
  if (this->meta_size() > 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteTagToArray(
      8,
      ::google::protobuf::internal::WireFormatLite::WIRETYPE_LENGTH_DELIMITED,
      target);
    target = ::google::protobuf::io::CodedOutputStream::WriteVarint32ToArray(
      _meta_cached_byte_size_, target);
  }
  for (int i = 0; i < this->meta_size(); i++) {
    target = ::google::protobuf::internal::WireFormatLite::
      WriteFloatNoTagToArray(this->meta(i), target);
  }

  if (!unknown_fields().empty()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        unknown_fields(), target);
  }
  return target;
}

int Sample::ByteSize() const {
  int total_size = 0;

  if (_has_bits_[0 / 32] & (0xffu << (0 % 32))) {
    // optional uint32 T = 1 [default = 100];
    if (has_t()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::UInt32Size(
          this->t());
    }

    // optional uint32 dX = 2;
    if (has_dx()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::UInt32Size(
          this->dx());
    }

    // optional uint32 dU = 3;
    if (has_du()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::UInt32Size(
          this->du());
    }

    // optional uint32 dO = 4;
    if (has_do_()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::UInt32Size(
          this->do_());
    }

  }
  // repeated float X = 5 [packed = true];
  {
    int data_size = 0;
    data_size = 4 * this->x_size();
    if (data_size > 0) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::Int32Size(data_size);
    }
    GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
    _x_cached_byte_size_ = data_size;
    GOOGLE_SAFE_CONCURRENT_WRITES_END();
    total_size += data_size;
  }

  // repeated float U = 6 [packed = true];
  {
    int data_size = 0;
    data_size = 4 * this->u_size();
    if (data_size > 0) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::Int32Size(data_size);
    }
    GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
    _u_cached_byte_size_ = data_size;
    GOOGLE_SAFE_CONCURRENT_WRITES_END();
    total_size += data_size;
  }

  // repeated float obs = 7 [packed = true];
  {
    int data_size = 0;
    data_size = 4 * this->obs_size();
    if (data_size > 0) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::Int32Size(data_size);
    }
    GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
    _obs_cached_byte_size_ = data_size;
    GOOGLE_SAFE_CONCURRENT_WRITES_END();
    total_size += data_size;
  }

  // repeated float meta = 8 [packed = true];
  {
    int data_size = 0;
    data_size = 4 * this->meta_size();
    if (data_size > 0) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::Int32Size(data_size);
    }
    GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
    _meta_cached_byte_size_ = data_size;
    GOOGLE_SAFE_CONCURRENT_WRITES_END();
    total_size += data_size;
  }

  if (!unknown_fields().empty()) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        unknown_fields());
  }
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = total_size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
  return total_size;
}

void Sample::MergeFrom(const ::google::protobuf::Message& from) {
  GOOGLE_CHECK_NE(&from, this);
  const Sample* source =
    ::google::protobuf::internal::dynamic_cast_if_available<const Sample*>(
      &from);
  if (source == NULL) {
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
    MergeFrom(*source);
  }
}

void Sample::MergeFrom(const Sample& from) {
  GOOGLE_CHECK_NE(&from, this);
  x_.MergeFrom(from.x_);
  u_.MergeFrom(from.u_);
  obs_.MergeFrom(from.obs_);
  meta_.MergeFrom(from.meta_);
  if (from._has_bits_[0 / 32] & (0xffu << (0 % 32))) {
    if (from.has_t()) {
      set_t(from.t());
    }
    if (from.has_dx()) {
      set_dx(from.dx());
    }
    if (from.has_du()) {
      set_du(from.du());
    }
    if (from.has_do_()) {
      set_do_(from.do_());
    }
  }
  mutable_unknown_fields()->MergeFrom(from.unknown_fields());
}

void Sample::CopyFrom(const ::google::protobuf::Message& from) {
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void Sample::CopyFrom(const Sample& from) {
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool Sample::IsInitialized() const {

  return true;
}

void Sample::Swap(Sample* other) {
  if (other != this) {
    std::swap(t_, other->t_);
    std::swap(dx_, other->dx_);
    std::swap(du_, other->du_);
    std::swap(do__, other->do__);
    x_.Swap(&other->x_);
    u_.Swap(&other->u_);
    obs_.Swap(&other->obs_);
    meta_.Swap(&other->meta_);
    std::swap(_has_bits_[0], other->_has_bits_[0]);
    _unknown_fields_.Swap(&other->_unknown_fields_);
    std::swap(_cached_size_, other->_cached_size_);
  }
}

::google::protobuf::Metadata Sample::GetMetadata() const {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::Metadata metadata;
  metadata.descriptor = Sample_descriptor_;
  metadata.reflection = Sample_reflection_;
  return metadata;
}


// @@protoc_insertion_point(namespace_scope)

}  // namespace gps

// @@protoc_insertion_point(global_scope)
