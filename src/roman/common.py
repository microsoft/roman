import numpy as np

################################################################
## common types
#################################################################
class Vec(object):
    '''
    Base class for types that wrap a one-dimensional numpy array.
    '''
    def __init__(self, size, dtype=np.float):
        self.array = np.zeros(size, dtype)

    @classmethod
    def fromarray(cls, array, clone = True):
        self = cls.__new__(cls)
        self.array = np.array(array) if clone else array
        return self

    def __getitem__(self, idx):
        return self.array[idx]

    def __setitem__(self, idx, val):
        self.array[idx] = val

    def __array__(self, dtype=None):
        return self.array

    def __iter__(self):
        return iter(self.array)

    def __len__(self):
        return len(self.array)
        
    def __str__(self):
        return np.array2string(self.array, separator=",")

    def __repr__(self):
        return type(self).__name__+"({s})".format(s = np.array2string(self.array, separator=","))

    def __iadd__(self, other):
        self.array += other
        return self

    def __isub__(self, other):
        self.array -= other
        return self

    def __add__(self, other):
        cls = self.__class__
        res = cls.__new__(cls)
        res.array = self.array + other
        return res

    def __sub__(self, other):
        cls = self.__class__
        res = cls.__new__(cls)
        res.array = self.array - other
        return res

    def __imul__(self, other):
        self.array *= other
        return self

    def __mul__(self, other):
        cls = self.__class__
        res = cls.__new__(cls)
        res.array = self.array * other
        return res
    
    def __idiv__(self, other):
        self.array /= other
        return self

    def __div__(self, other):
        cls = self.__class__
        res = cls.__new__(cls)
        res.array = self.array / other
        return res

    def __lt__(self, other):
        return np.all(self.array.__lt__(other))

    def __le__(self, other):
        return np.all(self.array.__le__(other))

    def __eq__(self, other):
        return np.all(self.array.__eq__(other))

    def __ne__(self, other):
        return np.any(self.array.__ne__(other))

    def __gt__(self, other):
        return np.all(self.array.__gt__(other))

    def __ge__(self, other):
        return np.all(self.array.__ge__(other))


################################################################
## socket operations
################################################################
def socket_send_retry(socket, buf, size = 0):
    total = 0
    if size == 0: size = len(buf)
    while size > total:
        sent = socket.send(buf[total:size])
        if sent == 0:
            return False
        total = total + sent
    return True

def socket_receive_retry(socket, buf, size = 0):
    view = memoryview(buf)
    if size == 0: size = len(buf)
    while size > 0:
        received = socket.recv_into(view, size)
        if received == 0:
            return False
        view = view[received:]
        size = size - received
    
    return True