#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to idaws_msgs__msg__Buoy

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Buoy {

    // This member is not documented.
    #[allow(missing_docs)]
    pub label: std::string::String,


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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::Buoy::default())
  }
}

impl rosidl_runtime_rs::Message for Buoy {
  type RmwMsg = super::msg::rmw::Buoy;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        label: msg.label.as_str().into(),
        confidence: msg.confidence,
        x_min: msg.x_min,
        y_min: msg.y_min,
        x_max: msg.x_max,
        y_max: msg.y_max,
        center_x: msg.center_x,
        center_y: msg.center_y,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        label: msg.label.as_str().into(),
      confidence: msg.confidence,
      x_min: msg.x_min,
      y_min: msg.y_min,
      x_max: msg.x_max,
      y_max: msg.y_max,
      center_x: msg.center_x,
      center_y: msg.center_y,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      label: msg.label.to_string(),
      confidence: msg.confidence,
      x_min: msg.x_min,
      y_min: msg.y_min,
      x_max: msg.x_max,
      y_max: msg.y_max,
      center_x: msg.center_x,
      center_y: msg.center_y,
    }
  }
}


// Corresponds to idaws_msgs__msg__BuoyArray

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct BuoyArray {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub buoys: Vec<super::msg::Buoy>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub frame_width: i32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub frame_height: i32,

}



impl Default for BuoyArray {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::BuoyArray::default())
  }
}

impl rosidl_runtime_rs::Message for BuoyArray {
  type RmwMsg = super::msg::rmw::BuoyArray;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        buoys: msg.buoys
          .into_iter()
          .map(|elem| super::msg::Buoy::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
        frame_width: msg.frame_width,
        frame_height: msg.frame_height,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
        buoys: msg.buoys
          .iter()
          .map(|elem| super::msg::Buoy::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      frame_width: msg.frame_width,
      frame_height: msg.frame_height,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      buoys: msg.buoys
          .into_iter()
          .map(super::msg::Buoy::from_rmw_message)
          .collect(),
      frame_width: msg.frame_width,
      frame_height: msg.frame_height,
    }
  }
}


// Corresponds to idaws_msgs__msg__Cluster
/// LiDAR taramasından ayrıştırılmış tek bir engel kümesi.
/// Koordinatlar araç gövde çerçevesinde: +x ileri, +y sol (metre).

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::Cluster::default())
  }
}

impl rosidl_runtime_rs::Message for Cluster {
  type RmwMsg = super::msg::rmw::Cluster;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        id: msg.id,
        center_x: msg.center_x,
        center_y: msg.center_y,
        range: msg.range,
        bearing: msg.bearing,
        min_range: msg.min_range,
        width: msg.width,
        point_count: msg.point_count,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      id: msg.id,
      center_x: msg.center_x,
      center_y: msg.center_y,
      range: msg.range,
      bearing: msg.bearing,
      min_range: msg.min_range,
      width: msg.width,
      point_count: msg.point_count,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      id: msg.id,
      center_x: msg.center_x,
      center_y: msg.center_y,
      range: msg.range,
      bearing: msg.bearing,
      min_range: msg.min_range,
      width: msg.width,
      point_count: msg.point_count,
    }
  }
}


// Corresponds to idaws_msgs__msg__ClusterArray

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ClusterArray {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub clusters: Vec<super::msg::Cluster>,

}



impl Default for ClusterArray {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::ClusterArray::default())
  }
}

impl rosidl_runtime_rs::Message for ClusterArray {
  type RmwMsg = super::msg::rmw::ClusterArray;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        clusters: msg.clusters
          .into_iter()
          .map(|elem| super::msg::Cluster::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
        clusters: msg.clusters
          .iter()
          .map(|elem| super::msg::Cluster::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      clusters: msg.clusters
          .into_iter()
          .map(super::msg::Cluster::from_rmw_message)
          .collect(),
    }
  }
}


