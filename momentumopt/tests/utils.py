from os import path, chdir, getcwd
import numpy as np


class NumDiffException(Exception):
    """ Raised when the difference bewtween the 2 Matrices is too high """
    pass


def assert_all_close(A, B, threshold):
    """ Assert analytical derivatives against NumDiff using the error norm.

    :param A: Matrix A
    :param B: Matrix B
    :param threshold: absolute tolerance
    """
    if not np.allclose(A, B, atol=threshold):
        value = np.linalg.norm(A - B)
        raise NumDiffException(
            "NumDiff exception, with residual of %.4g, above threshold %.4g" % (value, threshold))


class CD:
    """Context manager for changing the current working directory"""

    def __init__(self, newPath):
        self.newPath = path.expanduser(newPath)

    def __enter__(self):
        self.savedPath = getcwd()
        chdir(self.newPath)

    def __exit__(self, etype, value, traceback):
        chdir(self.savedPath)
