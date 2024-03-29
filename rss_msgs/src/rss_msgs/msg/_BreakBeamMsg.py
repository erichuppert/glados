"""autogenerated by genmsg_py from BreakBeamMsg.msg. Do not edit."""
import roslib.message
import struct


class BreakBeamMsg(roslib.message.Message):
  _md5sum = "5c99d1d4bed9929256313b1c7b10c3bd"
  _type = "rss_msgs/BreakBeamMsg"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """bool beamBroken
"""
  __slots__ = ['beamBroken']
  _slot_types = ['bool']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.
    
    The available fields are:
       beamBroken
    
    @param args: complete set of field values, in .msg order
    @param kwds: use keyword arguments corresponding to message field names
    to set specific fields. 
    """
    if args or kwds:
      super(BreakBeamMsg, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.beamBroken is None:
        self.beamBroken = False
    else:
      self.beamBroken = False

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    @param buff: buffer
    @type  buff: StringIO
    """
    try:
      buff.write(_struct_B.pack(self.beamBroken))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    @param str: byte array of serialized message
    @type  str: str
    """
    try:
      end = 0
      start = end
      end += 1
      (self.beamBroken,) = _struct_B.unpack(str[start:end])
      self.beamBroken = bool(self.beamBroken)
      return self
    except struct.error as e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    @param buff: buffer
    @type  buff: StringIO
    @param numpy: numpy python module
    @type  numpy module
    """
    try:
      buff.write(_struct_B.pack(self.beamBroken))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    @param str: byte array of serialized message
    @type  str: str
    @param numpy: numpy python module
    @type  numpy: module
    """
    try:
      end = 0
      start = end
      end += 1
      (self.beamBroken,) = _struct_B.unpack(str[start:end])
      self.beamBroken = bool(self.beamBroken)
      return self
    except struct.error as e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill

_struct_I = roslib.message.struct_I
_struct_B = struct.Struct("<B")
