import unittest

import Test00Core
import Test01GHVentilator

TestSickBaySetup():
  print("\n\nTesting SickBay\n")

class TestGVVentilator(unittest.TestCase):

  def TestInOrder(self):
    TestSickBaySetup()
    Test00Core()
    Test01GHVentilator()

  def TestInOrderWithLastSeamFirst(self):
    TestSickBaySetup()
    Test01GHVentilator()
    Test00Core()

if __name__ == '__main__':
    unittest.main()
