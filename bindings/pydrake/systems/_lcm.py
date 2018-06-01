from pydrake.systems.framework import Value


class PySerializer(SerializerInterface):
    def __init__(self, lcm_type):
        self._lcm_type = lcm_type

    def CreateDefaultValue(self):
        return Value.Make(self._lcm_type())

    def Deserialize(self, buffer):
        msg = self._lcm_type.decode(buffer)
        return Value.Make(msg)

    def Serialize(self, value):
        msg = value.get_value()
        assert isinstance(msg, self._lcm_type)
        return msg.encode()


# Attach methods (as descriptors).
@classmethod
def _make_lcm_system(cls, channel, lcm_type, lcm):
    return cls(channel, PySerializer(lcm_type), lcm)


LcmPublisherSystem.Make = _make_lcm_system
LcmSubscriberSystem.Make = _make_lcm_system
