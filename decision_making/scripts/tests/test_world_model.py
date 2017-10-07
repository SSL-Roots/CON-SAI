
import unittest

import sys,os
sys.path.append(os.pardir)

from world_model import WorldModel


class TestWorldModel(unittest.TestCase):
    
    def test_existing_friends_id(self):
        friends_id = WorldModel.existing_friends_id

        expected = 6
        actual = len(friends_id)

        self.assertEqual(expected, actual)


if __name__ == '__main__':
    unittest.main()
