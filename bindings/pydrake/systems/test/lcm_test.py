import pydrake.systems.lcm as mut

import unittest

from robotlocomotion import quaternion_t

from pydrake.lcm import DrakeMockLcm


msg = quaternion_t()
wxyz = (1, 2, 3, 4)
msg.w, msg.x, msg.y, msg.z = wxyz

class TestLcm(unittest.TestCase):
    def test_serializer(self):
        serializer = mut.PySerializer(quaternion_t)
        value = serializer.CreateDefaultValue()
        self.assertIsInstance(value.get_value(), quaternion_t)
        value = serializer.Deserialize(msg.encode())
        self.assertEqual(value.get_value().encode(), msg.encode())
        raw = serializer.Serialize(value)
        self.assertEqual(raw, msg.encode())
        print("YAWR")

    def test_subscriber(self):
        lcm = DrakeMockLcm()
        sub = mut.LcmSubscriberSystem.Make(
            channel="TEST_CHANNEL", lcm_type=quaternion_t, lcm=lcm)
        lcm.InduceSubscriberCallback(
            channel="TEST_CHANNEL", buffer=msg.encode())
        context = sub.CreateDefaultContext()
        output = system.AllocateOutput(context)
        sub.CalcOutput(context, output)
        actual = output.get_data(0).get_value()
        self.assertEqual(actual.encode(), msg.encode())

    # def test_publisher(self):
    #     lcm = DrakeMockLcm()
    #     pub = LcmPublisherSystem(
    #         channel="TEST_CHANNEL", lcm_type=quaternion_t, lcm=lcm)
    #     context = pub.CreateDefaultContext()
    #     context.FixInputPort(0, AbstractValue.Make(msg))
    #     sub.DoPublish(context)
    #     raw = dut.get_last_published_message("TEST_CHANNEL")
    #     self.assertEqual(raw, msg.encode())
