#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "idaws_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__idaws_msgs__msg__Buoy() -> *const std::ffi::c_void;
}

#[link(name = "idaws_msgs__rosidl_generator_c")]
extern "C" {
    fn idaws_msgs__msg__Buoy__init(msg: *mut Buoy) -> bool;
    fn idaws_msgs__msg__Buoy__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Buoy>, size: usize) -> bool;
    fn idaws_msgs__msg__Buoy__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Buoy>);
    fn idaws_msgs__msg__Buoy__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Buoy>, out_seq: *mut rosidl_runtime_rs::Sequence<Buoy>) -> bool;
}

// Corresponds to idaws_msgs__msg__Buoy
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Buoy {

    // This member is not documented.
    #[allow(missing_docs)]
    pub label: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub confidence: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub x_min: i32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub y_min: i32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub x_max: i32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub y_max: i32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub center_x: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub center_y: f32,

}



impl Default for Buoy {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !idaws_msgs__msg__Buoy__init(&mut msg as *mut _) {
        panic!("Call to idaws_msgs__msg__Buoy__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Buoy {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { idaws_msgs__msg__Buoy__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { idaws_msgs__msg__Buoy__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { idaws_msgs__msg__Buoy__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Buoy {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Buoy where Self: Sized {
  const TYPE_NAME: &'static str = "idaws_msgs/msg/Buoy";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__idaws_msgs__msg__Buoy() }
  }
}


#[link(name = "idaws_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__idaws_msgs__msg__BuoyArray() -> *const std::ffi::c_void;
}

#[link(name = "idaws_msgs__rosidl_generator_c")]
extern "C" {
    fn idaws_msgs__msg__BuoyArray__init(msg: *mut BuoyArray) -> bool;
    fn idaws_msgs__msg__BuoyArray__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<BuoyArray>, size: usize) -> bool;
    fn idaws_msgs__msg__BuoyArray__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<BuoyArray>);
    fn idaws_msgs__msg__BuoyArray__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<BuoyArray>, out_seq: *mut rosidl_runtime_rs::Sequence<BuoyArray>) -> bool;
}

// Corresponds to idaws_msgs__msg__BuoyArray
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct BuoyArray {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub buoys: rosidl_runtime_rs::Sequence<super::super::msg::rmw::Buoy>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub frame_width: i32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub frame_height: i32,

}



impl Default for BuoyArray {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !idaws_msgs__msg__BuoyArray__init(&mut msg as *mut _) {
        panic!("Call to idaws_msgs__msg__BuoyArray__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for BuoyArray {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { idaws_msgs__msg__BuoyArray__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { idaws_msgs__msg__BuoyArray__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { idaws_msgs__msg__BuoyArray__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for BuoyArray {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for BuoyArray where Self: Sized {
  const TYPE_NAME: &'static str = "idaws_msgs/msg/BuoyArray";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__idaws_msgs__msg__BuoyArray() }
  }
}


#[link(name = "idaws_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__idaws_msgs__msg__Cluster() -> *const std::ffi::c_void;
}

#[link(name = "idaws_msgs__rosidl_generator_c")]
extern "C" {
    fn idaws_msgs__msg__Cluster__init(msg: *mut Cluster) -> bool;
    fn idaws_msgs__msg__Cluster__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Cluster>, size: usize) -> bool;
    fn idaws_msgs__msg__Cluster__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Cluster>);
    fn idaws_msgs__msg__Cluster__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Cluster>, out_seq: *mut rosidl_runtime_rs::Sequence<Cluster>) -> bool;
}

// Corresponds to idaws_msgs__msg__Cluster
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// LiDAR taramasından ayrıştırılmış tek bir engel kümesi.
/// Koordinatlar araç gövde çerçevesinde: +x ileri, +y sol (metre).

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Cluster {

    // This member is not documented.
    #[allow(missing_docs)]
    pub id: i32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub center_x: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub center_y: f32,

    /// kümenin merkezine mesafe (m)
    pub range: f32,

    /// kümenin merkezinin kerteriz açısı (rad, +x'ten CCW)
    pub bearing: f32,

    /// kümedeki en yakın ışının mesafesi (m)
    pub min_range: f32,

    /// kümenin yaklaşık genişliği (m)
    pub width: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub point_count: i32,

}



impl Default for Cluster {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !idaws_msgs__msg__Cluster__init(&mut msg as *mut _) {
        panic!("Call to idaws_msgs__msg__Cluster__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Cluster {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { idaws_msgs__msg__Cluster__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { idaws_msgs__msg__Cluster__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { idaws_msgs__msg__Cluster__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Cluster {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Cluster where Self: Sized {
  const TYPE_NAME: &'static str = "idaws_msgs/msg/Cluster";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__idaws_msgs__msg__Cluster() }
  }
}


#[link(name = "idaws_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__idaws_msgs__msg__ClusterArray() -> *const std::ffi::c_void;
}

#[link(name = "idaws_msgs__rosidl_generator_c")]
extern "C" {
    fn idaws_msgs__msg__ClusterArray__init(msg: *mut ClusterArray) -> bool;
    fn idaws_msgs__msg__ClusterArray__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ClusterArray>, size: usize) -> bool;
    fn idaws_msgs__msg__ClusterArray__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ClusterArray>);
    fn idaws_msgs__msg__ClusterArray__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ClusterArray>, out_seq: *mut rosidl_runtime_rs::Sequence<ClusterArray>) -> bool;
}

// Corresponds to idaws_msgs__msg__ClusterArray
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ClusterArray {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub clusters: rosidl_runtime_rs::Sequence<super::super::msg::rmw::Cluster>,

}



impl Default for ClusterArray {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !idaws_msgs__msg__ClusterArray__init(&mut msg as *mut _) {
        panic!("Call to idaws_msgs__msg__ClusterArray__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ClusterArray {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { idaws_msgs__msg__ClusterArray__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { idaws_msgs__msg__ClusterArray__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { idaws_msgs__msg__ClusterArray__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ClusterArray {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ClusterArray where Self: Sized {
  const TYPE_NAME: &'static str = "idaws_msgs/msg/ClusterArray";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__idaws_msgs__msg__ClusterArray() }
  }
}


