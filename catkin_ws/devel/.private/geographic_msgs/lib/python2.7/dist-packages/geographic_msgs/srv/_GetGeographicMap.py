# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from geographic_msgs/GetGeographicMapRequest.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import geographic_msgs.msg

class GetGeographicMapRequest(genpy.Message):
  _md5sum = "505cc89008cb1745810d2ee4ea646d6e"
  _type = "geographic_msgs/GetGeographicMapRequest"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """# This service requests a region of a geographic map.

string url            # where to read map data

# Bounding box for the desired map.  If all zeros, provide all data
# available from the specified URL.
BoundingBox bounds


================================================================================
MSG: geographic_msgs/BoundingBox
# Geographic map bounding box. 
#
# The two GeoPoints denote diagonally opposite corners of the box.
#
# If min_pt.latitude is NaN, the bounding box is "global", matching
# any valid latitude, longitude and altitude.
#
# If min_pt.altitude is NaN, the bounding box is two-dimensional and
# matches any altitude within the specified latitude and longitude
# range.

GeoPoint min_pt         # lowest and most Southwestern corner
GeoPoint max_pt         # highest and most Northeastern corner

================================================================================
MSG: geographic_msgs/GeoPoint
# Geographic point, using the WGS 84 reference ellipsoid.

# Latitude [degrees]. Positive is north of equator; negative is south
# (-90 <= latitude <= +90).
float64 latitude

# Longitude [degrees]. Positive is east of prime meridian; negative is
# west (-180 <= longitude <= +180). At the poles, latitude is -90 or
# +90, and longitude is irrelevant, but must be in range.
float64 longitude

# Altitude [m]. Positive is above the WGS 84 ellipsoid (NaN if unspecified).
float64 altitude
"""
  __slots__ = ['url','bounds']
  _slot_types = ['string','geographic_msgs/BoundingBox']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       url,bounds

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(GetGeographicMapRequest, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.url is None:
        self.url = ''
      if self.bounds is None:
        self.bounds = geographic_msgs.msg.BoundingBox()
    else:
      self.url = ''
      self.bounds = geographic_msgs.msg.BoundingBox()

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self.url
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_6d().pack(_x.bounds.min_pt.latitude, _x.bounds.min_pt.longitude, _x.bounds.min_pt.altitude, _x.bounds.max_pt.latitude, _x.bounds.max_pt.longitude, _x.bounds.max_pt.altitude))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.bounds is None:
        self.bounds = geographic_msgs.msg.BoundingBox()
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.url = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.url = str[start:end]
      _x = self
      start = end
      end += 48
      (_x.bounds.min_pt.latitude, _x.bounds.min_pt.longitude, _x.bounds.min_pt.altitude, _x.bounds.max_pt.latitude, _x.bounds.max_pt.longitude, _x.bounds.max_pt.altitude,) = _get_struct_6d().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self.url
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_6d().pack(_x.bounds.min_pt.latitude, _x.bounds.min_pt.longitude, _x.bounds.min_pt.altitude, _x.bounds.max_pt.latitude, _x.bounds.max_pt.longitude, _x.bounds.max_pt.altitude))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.bounds is None:
        self.bounds = geographic_msgs.msg.BoundingBox()
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.url = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.url = str[start:end]
      _x = self
      start = end
      end += 48
      (_x.bounds.min_pt.latitude, _x.bounds.min_pt.longitude, _x.bounds.min_pt.altitude, _x.bounds.max_pt.latitude, _x.bounds.max_pt.longitude, _x.bounds.max_pt.altitude,) = _get_struct_6d().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_6d = None
def _get_struct_6d():
    global _struct_6d
    if _struct_6d is None:
        _struct_6d = struct.Struct("<6d")
    return _struct_6d
# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from geographic_msgs/GetGeographicMapResponse.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import geographic_msgs.msg
import std_msgs.msg
import uuid_msgs.msg

class GetGeographicMapResponse(genpy.Message):
  _md5sum = "0910332806c65953a4f4252eb780811a"
  _type = "geographic_msgs/GetGeographicMapResponse"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """
bool success          # true if the call succeeded
string status         # more details

# The requested map, its bounds may differ from the requested bounds.
GeographicMap map


================================================================================
MSG: geographic_msgs/GeographicMap
# Geographic map for a specified region.

Header header            # stamp specifies time
                         # frame_id (normally /map)

uuid_msgs/UniqueID id    # identifier for this map
BoundingBox  bounds      # 2D bounding box containing map

WayPoint[]   points      # way-points
MapFeature[] features    # map features
KeyValue[]   props       # map properties

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: uuid_msgs/UniqueID
# A universally unique identifier (UUID).
#
#  http://en.wikipedia.org/wiki/Universally_unique_identifier
#  http://tools.ietf.org/html/rfc4122.html

uint8[16] uuid

================================================================================
MSG: geographic_msgs/BoundingBox
# Geographic map bounding box. 
#
# The two GeoPoints denote diagonally opposite corners of the box.
#
# If min_pt.latitude is NaN, the bounding box is "global", matching
# any valid latitude, longitude and altitude.
#
# If min_pt.altitude is NaN, the bounding box is two-dimensional and
# matches any altitude within the specified latitude and longitude
# range.

GeoPoint min_pt         # lowest and most Southwestern corner
GeoPoint max_pt         # highest and most Northeastern corner

================================================================================
MSG: geographic_msgs/GeoPoint
# Geographic point, using the WGS 84 reference ellipsoid.

# Latitude [degrees]. Positive is north of equator; negative is south
# (-90 <= latitude <= +90).
float64 latitude

# Longitude [degrees]. Positive is east of prime meridian; negative is
# west (-180 <= longitude <= +180). At the poles, latitude is -90 or
# +90, and longitude is irrelevant, but must be in range.
float64 longitude

# Altitude [m]. Positive is above the WGS 84 ellipsoid (NaN if unspecified).
float64 altitude

================================================================================
MSG: geographic_msgs/WayPoint
# Way-point element for a geographic map.

uuid_msgs/UniqueID id   # Unique way-point identifier
GeoPoint   position     # Position relative to WGS 84 ellipsoid
KeyValue[] props        # Key/value properties for this point

================================================================================
MSG: geographic_msgs/KeyValue
# Geographic map tag (key, value) pair
#
# This is equivalent to diagnostic_msgs/KeyValue, repeated here to
# avoid introducing a trivial stack dependency.

string key                     # tag label
string value                   # corresponding value

================================================================================
MSG: geographic_msgs/MapFeature
# Geographic map feature.
#
# A list of WayPoint IDs for features like streets, highways, hiking
# trails, the outlines of buildings and parking lots in sequential
# order.
#
# Feature lists may also contain other feature lists as members.

uuid_msgs/UniqueID   id         # Unique feature identifier
uuid_msgs/UniqueID[] components # Sequence of feature components
KeyValue[] props                # Key/value properties for this feature
"""
  __slots__ = ['success','status','map']
  _slot_types = ['bool','string','geographic_msgs/GeographicMap']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       success,status,map

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(GetGeographicMapResponse, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.success is None:
        self.success = False
      if self.status is None:
        self.status = ''
      if self.map is None:
        self.map = geographic_msgs.msg.GeographicMap()
    else:
      self.success = False
      self.status = ''
      self.map = geographic_msgs.msg.GeographicMap()

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self.success
      buff.write(_get_struct_B().pack(_x))
      _x = self.status
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_3I().pack(_x.map.header.seq, _x.map.header.stamp.secs, _x.map.header.stamp.nsecs))
      _x = self.map.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self.map.id.uuid
      # - if encoded as a list instead, serialize as bytes instead of string
      if type(_x) in [list, tuple]:
        buff.write(_get_struct_16B().pack(*_x))
      else:
        buff.write(_get_struct_16s().pack(_x))
      _x = self
      buff.write(_get_struct_6d().pack(_x.map.bounds.min_pt.latitude, _x.map.bounds.min_pt.longitude, _x.map.bounds.min_pt.altitude, _x.map.bounds.max_pt.latitude, _x.map.bounds.max_pt.longitude, _x.map.bounds.max_pt.altitude))
      length = len(self.map.points)
      buff.write(_struct_I.pack(length))
      for val1 in self.map.points:
        _v1 = val1.id
        _x = _v1.uuid
        # - if encoded as a list instead, serialize as bytes instead of string
        if type(_x) in [list, tuple]:
          buff.write(_get_struct_16B().pack(*_x))
        else:
          buff.write(_get_struct_16s().pack(_x))
        _v2 = val1.position
        _x = _v2
        buff.write(_get_struct_3d().pack(_x.latitude, _x.longitude, _x.altitude))
        length = len(val1.props)
        buff.write(_struct_I.pack(length))
        for val2 in val1.props:
          _x = val2.key
          length = len(_x)
          if python3 or type(_x) == unicode:
            _x = _x.encode('utf-8')
            length = len(_x)
          buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
          _x = val2.value
          length = len(_x)
          if python3 or type(_x) == unicode:
            _x = _x.encode('utf-8')
            length = len(_x)
          buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      length = len(self.map.features)
      buff.write(_struct_I.pack(length))
      for val1 in self.map.features:
        _v3 = val1.id
        _x = _v3.uuid
        # - if encoded as a list instead, serialize as bytes instead of string
        if type(_x) in [list, tuple]:
          buff.write(_get_struct_16B().pack(*_x))
        else:
          buff.write(_get_struct_16s().pack(_x))
        length = len(val1.components)
        buff.write(_struct_I.pack(length))
        for val2 in val1.components:
          _x = val2.uuid
          # - if encoded as a list instead, serialize as bytes instead of string
          if type(_x) in [list, tuple]:
            buff.write(_get_struct_16B().pack(*_x))
          else:
            buff.write(_get_struct_16s().pack(_x))
        length = len(val1.props)
        buff.write(_struct_I.pack(length))
        for val2 in val1.props:
          _x = val2.key
          length = len(_x)
          if python3 or type(_x) == unicode:
            _x = _x.encode('utf-8')
            length = len(_x)
          buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
          _x = val2.value
          length = len(_x)
          if python3 or type(_x) == unicode:
            _x = _x.encode('utf-8')
            length = len(_x)
          buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      length = len(self.map.props)
      buff.write(_struct_I.pack(length))
      for val1 in self.map.props:
        _x = val1.key
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
        _x = val1.value
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.map is None:
        self.map = geographic_msgs.msg.GeographicMap()
      end = 0
      start = end
      end += 1
      (self.success,) = _get_struct_B().unpack(str[start:end])
      self.success = bool(self.success)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.status = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.status = str[start:end]
      _x = self
      start = end
      end += 12
      (_x.map.header.seq, _x.map.header.stamp.secs, _x.map.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.map.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.map.header.frame_id = str[start:end]
      start = end
      end += 16
      self.map.id.uuid = str[start:end]
      _x = self
      start = end
      end += 48
      (_x.map.bounds.min_pt.latitude, _x.map.bounds.min_pt.longitude, _x.map.bounds.min_pt.altitude, _x.map.bounds.max_pt.latitude, _x.map.bounds.max_pt.longitude, _x.map.bounds.max_pt.altitude,) = _get_struct_6d().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.map.points = []
      for i in range(0, length):
        val1 = geographic_msgs.msg.WayPoint()
        _v4 = val1.id
        start = end
        end += 16
        _v4.uuid = str[start:end]
        _v5 = val1.position
        _x = _v5
        start = end
        end += 24
        (_x.latitude, _x.longitude, _x.altitude,) = _get_struct_3d().unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        val1.props = []
        for i in range(0, length):
          val2 = geographic_msgs.msg.KeyValue()
          start = end
          end += 4
          (length,) = _struct_I.unpack(str[start:end])
          start = end
          end += length
          if python3:
            val2.key = str[start:end].decode('utf-8', 'rosmsg')
          else:
            val2.key = str[start:end]
          start = end
          end += 4
          (length,) = _struct_I.unpack(str[start:end])
          start = end
          end += length
          if python3:
            val2.value = str[start:end].decode('utf-8', 'rosmsg')
          else:
            val2.value = str[start:end]
          val1.props.append(val2)
        self.map.points.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.map.features = []
      for i in range(0, length):
        val1 = geographic_msgs.msg.MapFeature()
        _v6 = val1.id
        start = end
        end += 16
        _v6.uuid = str[start:end]
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        val1.components = []
        for i in range(0, length):
          val2 = uuid_msgs.msg.UniqueID()
          start = end
          end += 16
          val2.uuid = str[start:end]
          val1.components.append(val2)
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        val1.props = []
        for i in range(0, length):
          val2 = geographic_msgs.msg.KeyValue()
          start = end
          end += 4
          (length,) = _struct_I.unpack(str[start:end])
          start = end
          end += length
          if python3:
            val2.key = str[start:end].decode('utf-8', 'rosmsg')
          else:
            val2.key = str[start:end]
          start = end
          end += 4
          (length,) = _struct_I.unpack(str[start:end])
          start = end
          end += length
          if python3:
            val2.value = str[start:end].decode('utf-8', 'rosmsg')
          else:
            val2.value = str[start:end]
          val1.props.append(val2)
        self.map.features.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.map.props = []
      for i in range(0, length):
        val1 = geographic_msgs.msg.KeyValue()
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.key = str[start:end].decode('utf-8', 'rosmsg')
        else:
          val1.key = str[start:end]
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.value = str[start:end].decode('utf-8', 'rosmsg')
        else:
          val1.value = str[start:end]
        self.map.props.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self.success
      buff.write(_get_struct_B().pack(_x))
      _x = self.status
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_3I().pack(_x.map.header.seq, _x.map.header.stamp.secs, _x.map.header.stamp.nsecs))
      _x = self.map.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self.map.id.uuid
      # - if encoded as a list instead, serialize as bytes instead of string
      if type(_x) in [list, tuple]:
        buff.write(_get_struct_16B().pack(*_x))
      else:
        buff.write(_get_struct_16s().pack(_x))
      _x = self
      buff.write(_get_struct_6d().pack(_x.map.bounds.min_pt.latitude, _x.map.bounds.min_pt.longitude, _x.map.bounds.min_pt.altitude, _x.map.bounds.max_pt.latitude, _x.map.bounds.max_pt.longitude, _x.map.bounds.max_pt.altitude))
      length = len(self.map.points)
      buff.write(_struct_I.pack(length))
      for val1 in self.map.points:
        _v7 = val1.id
        _x = _v7.uuid
        # - if encoded as a list instead, serialize as bytes instead of string
        if type(_x) in [list, tuple]:
          buff.write(_get_struct_16B().pack(*_x))
        else:
          buff.write(_get_struct_16s().pack(_x))
        _v8 = val1.position
        _x = _v8
        buff.write(_get_struct_3d().pack(_x.latitude, _x.longitude, _x.altitude))
        length = len(val1.props)
        buff.write(_struct_I.pack(length))
        for val2 in val1.props:
          _x = val2.key
          length = len(_x)
          if python3 or type(_x) == unicode:
            _x = _x.encode('utf-8')
            length = len(_x)
          buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
          _x = val2.value
          length = len(_x)
          if python3 or type(_x) == unicode:
            _x = _x.encode('utf-8')
            length = len(_x)
          buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      length = len(self.map.features)
      buff.write(_struct_I.pack(length))
      for val1 in self.map.features:
        _v9 = val1.id
        _x = _v9.uuid
        # - if encoded as a list instead, serialize as bytes instead of string
        if type(_x) in [list, tuple]:
          buff.write(_get_struct_16B().pack(*_x))
        else:
          buff.write(_get_struct_16s().pack(_x))
        length = len(val1.components)
        buff.write(_struct_I.pack(length))
        for val2 in val1.components:
          _x = val2.uuid
          # - if encoded as a list instead, serialize as bytes instead of string
          if type(_x) in [list, tuple]:
            buff.write(_get_struct_16B().pack(*_x))
          else:
            buff.write(_get_struct_16s().pack(_x))
        length = len(val1.props)
        buff.write(_struct_I.pack(length))
        for val2 in val1.props:
          _x = val2.key
          length = len(_x)
          if python3 or type(_x) == unicode:
            _x = _x.encode('utf-8')
            length = len(_x)
          buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
          _x = val2.value
          length = len(_x)
          if python3 or type(_x) == unicode:
            _x = _x.encode('utf-8')
            length = len(_x)
          buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      length = len(self.map.props)
      buff.write(_struct_I.pack(length))
      for val1 in self.map.props:
        _x = val1.key
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
        _x = val1.value
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.map is None:
        self.map = geographic_msgs.msg.GeographicMap()
      end = 0
      start = end
      end += 1
      (self.success,) = _get_struct_B().unpack(str[start:end])
      self.success = bool(self.success)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.status = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.status = str[start:end]
      _x = self
      start = end
      end += 12
      (_x.map.header.seq, _x.map.header.stamp.secs, _x.map.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.map.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.map.header.frame_id = str[start:end]
      start = end
      end += 16
      self.map.id.uuid = str[start:end]
      _x = self
      start = end
      end += 48
      (_x.map.bounds.min_pt.latitude, _x.map.bounds.min_pt.longitude, _x.map.bounds.min_pt.altitude, _x.map.bounds.max_pt.latitude, _x.map.bounds.max_pt.longitude, _x.map.bounds.max_pt.altitude,) = _get_struct_6d().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.map.points = []
      for i in range(0, length):
        val1 = geographic_msgs.msg.WayPoint()
        _v10 = val1.id
        start = end
        end += 16
        _v10.uuid = str[start:end]
        _v11 = val1.position
        _x = _v11
        start = end
        end += 24
        (_x.latitude, _x.longitude, _x.altitude,) = _get_struct_3d().unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        val1.props = []
        for i in range(0, length):
          val2 = geographic_msgs.msg.KeyValue()
          start = end
          end += 4
          (length,) = _struct_I.unpack(str[start:end])
          start = end
          end += length
          if python3:
            val2.key = str[start:end].decode('utf-8', 'rosmsg')
          else:
            val2.key = str[start:end]
          start = end
          end += 4
          (length,) = _struct_I.unpack(str[start:end])
          start = end
          end += length
          if python3:
            val2.value = str[start:end].decode('utf-8', 'rosmsg')
          else:
            val2.value = str[start:end]
          val1.props.append(val2)
        self.map.points.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.map.features = []
      for i in range(0, length):
        val1 = geographic_msgs.msg.MapFeature()
        _v12 = val1.id
        start = end
        end += 16
        _v12.uuid = str[start:end]
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        val1.components = []
        for i in range(0, length):
          val2 = uuid_msgs.msg.UniqueID()
          start = end
          end += 16
          val2.uuid = str[start:end]
          val1.components.append(val2)
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        val1.props = []
        for i in range(0, length):
          val2 = geographic_msgs.msg.KeyValue()
          start = end
          end += 4
          (length,) = _struct_I.unpack(str[start:end])
          start = end
          end += length
          if python3:
            val2.key = str[start:end].decode('utf-8', 'rosmsg')
          else:
            val2.key = str[start:end]
          start = end
          end += 4
          (length,) = _struct_I.unpack(str[start:end])
          start = end
          end += length
          if python3:
            val2.value = str[start:end].decode('utf-8', 'rosmsg')
          else:
            val2.value = str[start:end]
          val1.props.append(val2)
        self.map.features.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.map.props = []
      for i in range(0, length):
        val1 = geographic_msgs.msg.KeyValue()
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.key = str[start:end].decode('utf-8', 'rosmsg')
        else:
          val1.key = str[start:end]
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.value = str[start:end].decode('utf-8', 'rosmsg')
        else:
          val1.value = str[start:end]
        self.map.props.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_16B = None
def _get_struct_16B():
    global _struct_16B
    if _struct_16B is None:
        _struct_16B = struct.Struct("<16B")
    return _struct_16B
_struct_16s = None
def _get_struct_16s():
    global _struct_16s
    if _struct_16s is None:
        _struct_16s = struct.Struct("<16s")
    return _struct_16s
_struct_3I = None
def _get_struct_3I():
    global _struct_3I
    if _struct_3I is None:
        _struct_3I = struct.Struct("<3I")
    return _struct_3I
_struct_3d = None
def _get_struct_3d():
    global _struct_3d
    if _struct_3d is None:
        _struct_3d = struct.Struct("<3d")
    return _struct_3d
_struct_6d = None
def _get_struct_6d():
    global _struct_6d
    if _struct_6d is None:
        _struct_6d = struct.Struct("<6d")
    return _struct_6d
_struct_B = None
def _get_struct_B():
    global _struct_B
    if _struct_B is None:
        _struct_B = struct.Struct("<B")
    return _struct_B
class GetGeographicMap(object):
  _type          = 'geographic_msgs/GetGeographicMap'
  _md5sum = 'c0278e653eee0ad79600510650e7be39'
  _request_class  = GetGeographicMapRequest
  _response_class = GetGeographicMapResponse
