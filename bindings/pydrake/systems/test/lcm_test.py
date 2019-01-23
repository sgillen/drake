import pydrake.systems.lcm as mut

import collections
import unittest
import time
from threading import Thread

import numpy as np
from six import text_type as unicode

from robotlocomotion import header_t, quaternion_t
from lcm import LCM

from pydrake.lcm import DrakeLcm, DrakeMockLcm
from pydrake.systems.framework import (AbstractValue, BasicVector,
                                       DiagramBuilder, LeafSystem)
from pydrake.systems.primitives import ConstantVectorSource, LogOutput


# TODO(eric.cousieau): Move this to more generic code when another piece of
# code uses it.
def lcm_to_json(message):
    def helper(thing):
        if type(thing) in (int, float, np.float64, str, unicode):
            return thing
        if type(thing) in (list,):
            return map(helper, [x for x in thing])
        result = collections.OrderedDict()
        for field in thing.__slots__:
            value = getattr(thing, field)
            result[field] = helper(value)
        return result
    return helper(message)


class DummySys(LeafSystem):
    def __init__(self):
        LeafSystem.__init__(self)
        self._DeclareAbstractInputPort("header_t",
                                       AbstractValue.Make(header_t))
        self._DeclareVectorOutputPort(BasicVector(1), self.CalcTimestamp)

    def CalcTimestamp(self, context, output):
        x = self.EvalAbstractInput(context, 0).get_value()
        y = output.get_mutable_value()
        y[:] = x.utime / 1e6


class TestSystemsLcm(unittest.TestCase):
    def _model_message(self):
        message = quaternion_t()
        message.w, message.x, message.y, message.z = (1, 2, 3, 4)
        return message

    def _model_value_cpp(self):
        serializer = mut._Serializer_[quaternion_t]()
        model_message = self._model_message()
        model_value = serializer.CreateDefaultValue()
        serializer.Deserialize(model_message.encode(), model_value)
        return model_value

    def _cpp_value_to_py_message(self, value):
        serializer = mut._Serializer_[quaternion_t]()
        raw = serializer.Serialize(value)
        return quaternion_t.decode(raw)

    def assert_lcm_equal(self, actual, expected):
        self.assertIsInstance(actual, type(expected))
        self.assertDictEqual(lcm_to_json(actual), lcm_to_json(expected))

    def assert_lcm_not_equal(self, actual, expected):
        self.assertIsInstance(actual, type(expected))
        self.assertNotEqual(lcm_to_json(actual), lcm_to_json(expected))

    def test_serializer(self):
        dut = mut.PySerializer(quaternion_t)
        model_message = self._model_message()
        value = dut.CreateDefaultValue()
        self.assert_lcm_not_equal(value.get_value(), model_message)
        # Check deserialization.
        dut.Deserialize(model_message.encode(), value)
        self.assert_lcm_equal(value.get_value(), model_message)
        # Check serialization.
        raw = dut.Serialize(value)
        reconstruct = quaternion_t.decode(raw)
        self.assert_lcm_equal(reconstruct, model_message)

    def test_serializer_cpp(self):
        # Tests relevant portions of API.
        model_message = self._model_message()
        model_value = self._model_value_cpp()
        self.assert_lcm_equal(
            self._cpp_value_to_py_message(model_value), model_message)

    def _calc_output(self, dut):
        context = dut.CreateDefaultContext()
        output = dut.AllocateOutput()
        dut.CopyLatestMessageInto(context.get_mutable_state())
        dut.CalcOutput(context, output)
        actual = output.get_data(0)
        return actual

    def test_utime_to_seconds(self):
        serial = mut.PySerializer(header_t)
        dut = mut.PyUtimeMessageToSeconds(header_t)
        model = header_t()
        model.utime, model.seq, model.frame_name = (1e6, 2, "header")
        value = serial.CreateDefaultValue()
        serial.Deserialize(model.encode(), value)
        time = dut.GetTimeInSeconds(value)
        self.assertEqual(time, 1)

    def test_subscriber(self):
        lcm = DrakeMockLcm()
        dut = mut.LcmSubscriberSystem.Make(
            channel="TEST_CHANNEL", lcm_type=quaternion_t, lcm=lcm)
        model_message = self._model_message()
        lcm.InduceSubscriberCallback(
            channel="TEST_CHANNEL", buffer=model_message.encode())
        actual_message = self._calc_output(dut).get_value()
        self.assert_lcm_equal(actual_message, model_message)

    def test_subscriber_cpp(self):
        lcm = DrakeMockLcm()
        dut = mut.LcmSubscriberSystem.Make(
            channel="TEST_CHANNEL", lcm_type=quaternion_t, lcm=lcm,
            use_cpp_serializer=True)
        model_message = self._model_message()
        lcm.InduceSubscriberCallback(
            channel="TEST_CHANNEL", buffer=model_message.encode())
        actual_message = self._cpp_value_to_py_message(self._calc_output(dut))
        self.assert_lcm_equal(actual_message, model_message)

    def _fix_and_publish(self, dut, value):
        context = dut.CreateDefaultContext()
        context.FixInputPort(0, value)
        dut.Publish(context)

    def test_publisher(self):
        lcm = DrakeMockLcm()
        dut = mut.LcmPublisherSystem.Make(
            channel="TEST_CHANNEL", lcm_type=quaternion_t, lcm=lcm,
            publish_period=0.1)
        model_message = self._model_message()
        self._fix_and_publish(dut, AbstractValue.Make(model_message))
        raw = lcm.get_last_published_message("TEST_CHANNEL")
        actual_message = quaternion_t.decode(raw)
        self.assert_lcm_equal(actual_message, model_message)

    def test_publisher_cpp(self):
        lcm = DrakeMockLcm()
        dut = mut.LcmPublisherSystem.Make(
            channel="TEST_CHANNEL", lcm_type=quaternion_t, lcm=lcm,
            use_cpp_serializer=True)
        model_message = self._model_message()
        model_value = self._model_value_cpp()
        self._fix_and_publish(dut, model_value)
        raw = lcm.get_last_published_message("TEST_CHANNEL")
        actual_message = quaternion_t.decode(raw)
        self.assert_lcm_equal(actual_message, model_message)

    def test_lcm_driven_loop(self):
        kStart = 3
        kEnd = 10
        kLcmUrl = "udpm://239.255.76.67:7668"

        lcm = DrakeLcm(kLcmUrl)
        builder = DiagramBuilder()
        sub = builder.AddSystem(mut.LcmSubscriberSystem.Make("TEST_LOOP",
                                                             header_t, lcm))
        dummy = builder.AddSystem(DummySys())
        logger = LogOutput(dummy.get_output_port(0), builder)
        # logger.set_forced_publish_only()
        builder.Connect(sub.get_output_port(0), dummy.get_input_port(0))
        diagram = builder.Build()

        utime = mut.PyUtimeMessageToSeconds(header_t)
        dut = mut.LcmDrivenLoop(diagram, sub, None, lcm, utime)
        dut.set_publish_on_every_received_message(True)

        import sys, trace
        sys.stdout = sys.stderr
        from functools import partial
        tracer = trace.Trace(trace=1, count=0, ignoredirs=["/usr", sys.prefix])

        def publish():
            print("Hello")
            lcm = LCM(kLcmUrl)
            msg = header_t()
            kSleepSec = 0.1
            time.sleep(kSleepSec)

            # for i in range(kStart, kEnd+1):
            if True:
                i = 1
                msg.utime = 1e6 * i
                data = msg.encode()
                lcm.publish("TEST_LOOP", data)
                # time.sleep(kSleepSec)

        thread = Thread(target=publish)  #partial(tracer.runfunc, publish))
        thread.start()

        first_msg = dut.WaitForMessage()
        # utime_recovered = dut.get_message_to_time_converter()
        # msg_time = utime_recovered.GetTimeInSeconds(first_msg)

        thread.join()

    def test_connect_lcm_scope(self):
        builder = DiagramBuilder()
        source = builder.AddSystem(ConstantVectorSource(np.zeros(4)))
        mut.ConnectLcmScope(src=source.get_output_port(0),
                            channel="TEST_CHANNEL",
                            builder=builder,
                            lcm=DrakeMockLcm())
