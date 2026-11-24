import unittest

from t800_ros_api.state import LatestMessageCache


class CacheTests(unittest.TestCase):
    def test_get_missing_topic(self):
        self.assertIsNone(LatestMessageCache().latest('missing', 2.0))

    def test_cached_snapshot_is_independent(self):
        cache = LatestMessageCache()
        content = {'data': 'ARMED'}
        cache.update('controller/status', '/my_ros2/controller/status',
                     'std_msgs/msg/String', content, {'state': 'ARMED'})
        content['data'] = 'CHANGED'
        result = cache.latest('controller/status', 5.0)
        self.assertEqual(result['data']['data'], 'ARMED')
        self.assertFalse(result['stale'])
        self.assertGreaterEqual(result['age_ms'], 0)
        result['parsed']['state'] = 'CHANGED'
        self.assertEqual(cache.latest('controller/status', 5)['parsed']['state'], 'ARMED')

    def test_stale_snapshot_is_returned(self):
        cache = LatestMessageCache()
        cache.update('localization/status', '/my_ros1/localization/status',
                     'std_msgs/msg/String', {'data': 'COLLECTING'})
        result = cache.latest('localization/status', -1.0)
        self.assertTrue(result['stale'])


if __name__ == '__main__':
    unittest.main()
