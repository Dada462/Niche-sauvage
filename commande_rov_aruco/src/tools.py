import numpy as np

def QR_to_cage(QR_id,X_QR):
    """
    Takes the QR code id and the Position of the robot in the QR code's frame
    Example: QR_id=1; X_QR=np.array([1,69,4.5]) or [1,69,4.5] etc.
    """
    P034=[[0,-1,0],[-1,0,-.5],[-1,-2,-.5]]
    P1=[-.5,0,-.5]
    P2=[-.5,-1.5,-.5]
    if QR_id in [0,3,4]:
        mapping={0:0,3:1,4:2}
        i=mapping[QR_id]
        Px,Py,Pz=P034[i]
        S=np.array([[0,0,-1,Px],
                    [-1,0,0,Py],
                    [0,1,0,Pz],
                    [0,0,0,1]])
        return (S@np.array([*X_QR,1]))[:3]
    elif QR_id==1:
        Px,Py,Pz=P1
        S=np.array([[1,0,0,Px],
                    [0,0,-1,Py],
                    [0,1,0,Pz],
                    [0,0,0,1]])
        return (S@np.array([*X_QR,1]))[:3]
    elif QR_id==2:
        Px,Py,Pz=P2
        S=np.array([[-1,0,0,Px],
                    [0,0,1,Py],
                    [0,1,0,Pz],
                    [0,0,0,1]])
        return (S@np.array([*X_QR,1]))[:3]
    else:
        raise TypeError("QR code ids between 0 and 4 are allowed")

if __name__=='__main__':
    QR_id=2
    X_QR=[0,0,0]
    pos=QR_to_cage(QR_id,X_QR)
    print('Position converted from QR code '+str(QR_id),':',pos)