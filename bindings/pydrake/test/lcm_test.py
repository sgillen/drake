import unittest

from pydrake.lcm import DrakeLcm, DrakeLcmInterface, DrakeMockLcm

from robotlocomotion import quaternion_t


class TestLcm(unittest.TestCase):
    def test_lcm(self):
        dut = DrakeLcm()
        self.assertIsInstance(dut, DrakeLcmInterface)
        # Quickly start and stop the receiving thread.
        dut.StartReceiveThread()
        dut.StopReceiveThread()

    def test_mock_lcm(self):
        dut = DrakeMockLcm()
        self.assertIsInstance(dut, DrakeLcmInterface)

        msg = quaternion_t()
        msg.w, msg.x, msg.y, msg.z = (1, 2, 3, 4)

        called = [False]

        def handler(raw):
            msg = quaternion_t.decode(raw)
            self.assertTupleEqual(
                (msg.w, msg.x, msg.y, msg.z), (1, 2, 3, 4))
            # Update capture.
            called[0] = True

        print(type(msg.encode()))
        print(repr(msg.encode()))
        dut.Publish(channel="TEST_CHANNEL", buffer=msg.encode())
        raw = dut.get_last_published_message("TEST_CHANNEL")
        self.assertEqual(raw, msg.encode())

        dut.Subscribe(channel="TEST_CHANNEL", handler=handler)
        self.assertTrue(called[0])
