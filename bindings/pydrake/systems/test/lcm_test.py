import pydrake.systems.lcm as mut

import unittest

from robotlocomotion import quaternion_t

from pydrake.lcm import DrakeMockLcm
from pydrake.systems.framework import AbstractValue


class TestSystemsLcm(unittest.TestCase):
    def _model_message(self):
        message = quaternion_t()
        message.w, message.x, message.y, message.z = (1, 2, 3, 4)
        return message

    def _assert_model_message(self, message):
        self.assertIsInstance(message, quaternion_t)
        self.assertTupleEqual(
            (message.w, message.x, message.y, message.z),
            (1, 2, 3, 4))

    def _assert_not_model_message(self, message):
        self.assertIsInstance(message, quaternion_t)
        self.assertNotEqual(
            (message.w, message.x, message.y, message.z),
            (1, 2, 3, 4))

    def test_serializer(self):
        dut = mut.PySerializer(quaternion_t)
        value = dut.CreateDefaultValue()
        self._assert_not_model_message(value.get_value())
        # Check deserialization.
        model = self._model_message()
        dut.Deserialize(model.encode(), value)
        self._assert_model_message(value.get_value())
        # CHeck serialization.
        raw = dut.Serialize(value)
        reconstruct = quaternion_t.decode(raw)
        self._assert_model_message(reconstruct)

    def test_subscriber(self):
        lcm = DrakeMockLcm()
        dut = mut.LcmSubscriberSystem.Make(
            channel="TEST_CHANNEL", lcm_type=quaternion_t, lcm=lcm)
        model = self._model_message()
        lcm.InduceSubscriberCallback(
            channel="TEST_CHANNEL", buffer=model.encode())
        context = dut.CreateDefaultContext()
        output = dut.AllocateOutput()
        dut.CalcOutput(context, output)
        actual = output.get_data(0).get_value()
        self._assert_model_message(actual)

    def test_publisher(self):
        lcm = DrakeMockLcm()
        dut = mut.LcmPublisherSystem.Make(
            channel="TEST_CHANNEL", lcm_type=quaternion_t, lcm=lcm)
        model = self._model_message()
        dut.set_publish_period(period=0.1)
        context = dut.CreateDefaultContext()
        context.FixInputPort(0, AbstractValue.Make(model))
        dut.Publish(context)
        raw = lcm.get_last_published_message("TEST_CHANNEL")
        value = quaternion_t.decode(raw)
        self._assert_model_message(value)
