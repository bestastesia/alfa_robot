from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from v3_fine_collision_retry import search_with_fine_retry


class FakeValidity:
    def mask(self, values):
        return np.asarray(values,dtype=bool)


class FineCollisionRetryTest(unittest.TestCase):
    def test_success_keeps_original_resolution_and_does_not_retry(self):
        validity=FakeValidity()
        calls=[]
        def search(*args,**kwargs):
            calls.append(validity.search_edge_resolution)
            validity.mask([True,True])
            return 'path',{}
        path,stats=search_with_fine_retry(search,None,None,None,None,validity,2.,1)
        self.assertEqual(path,'path')
        self.assertEqual(calls,[.05])
        self.assertFalse(stats['fine_collision_retry']['triggered'])
        self.assertNotIn('mask',validity.__dict__)

    def test_fine_collision_restarts_denser_with_remaining_budget(self):
        validity=FakeValidity()
        calls=[]
        def search(start,goals,lower,upper,instance,budget,seed):
            calls.append((instance.search_edge_resolution,budget))
            if len(calls)==1:
                instance.mask([True,False])
            return 'safe_path',{}
        path,stats=search_with_fine_retry(search,None,None,None,None,validity,2.,1)
        self.assertEqual(path,'safe_path')
        self.assertEqual([item[0] for item in calls],[.05,.03])
        self.assertLessEqual(calls[1][1],calls[0][1])
        self.assertEqual(stats['fine_collision_retry']['attempts'][0]['first_bad_frame'],2)

    def test_no_path_without_fine_collision_does_not_retry(self):
        validity=FakeValidity()
        calls=[]
        def search(*args,**kwargs):
            calls.append(1)
            return None,dict(failure_reason='search_timeout')
        path,stats=search_with_fine_retry(search,None,None,None,None,validity,2.,1)
        self.assertIsNone(path)
        self.assertEqual(len(calls),1)
        self.assertFalse(stats['fine_collision_retry']['triggered'])

    def test_other_error_propagates_and_restores_validity(self):
        validity=FakeValidity()
        validity.search_edge_resolution=.07
        def search(*args,**kwargs):
            raise ValueError('unrelated error')
        with self.assertRaises(ValueError):
            search_with_fine_retry(search,None,None,None,None,validity,2.,1)
        self.assertEqual(validity.search_edge_resolution,.07)


if __name__=='__main__':
    unittest.main()
