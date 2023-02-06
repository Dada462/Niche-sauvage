import pyqtgraph as pg
from pyqtgraph.Qt import QtCore
import numpy as np
from pyqtgraph.Qt import QtCore, QtWidgets,QtGui
from PyQt5.QtGui import QPainter, QBrush, QPen
import sys
import rospy
from std_msgs.msg import Float64
import threading
from usbl.msg import Usbl
from sensor_msgs.msg import Imu


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, *args, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)

        self.graphWidget=pg.GraphicsLayoutWidget()
        self.setCentralWidget(self.graphWidget)
        self.resize(850,600)
        self.setWindowTitle('Mission Displayer')

        #Test data
        self.usbl_data=None
        self.heading=None
        t=np.linspace(0,3.15,100)
        self.x = np.cos(t)
        self.y = np.sin(t)
        
        self.vehicle = pg.ArrowItem(angle=0, tipAngle=30, baseAngle=-30, headLen=40, tailLen=None,brush=(40,138,241,180))
        self.vehicle.setPos(0,0)
        
        pen = pg.mkPen(color='#288af1', width=3, style=QtCore.Qt.DashLine)
        
        #Creating the widgets
        self.graphWidget.setBackground('w')
        # self.graph_item = pg.GraphItem(pen=pen,symbol='star',symbolSize=30, symbolBrush='#288af1',name='Sensor 1')
        self.graph_item = pg.GraphItem(pen=pen)
        self.p=self.graphWidget.addPlot()
        
        #Setting the plot
        self.p.setYRange(-1,1)
        self.p.setLabel('left', 'y position (m)', **{'color':'r', 'font-size':'20px'})
        self.p.setLabel('bottom', 'x position (m)', **{'color':'r', 'font-size':'20px'})
        self.p.addLegend()
        self.p.showGrid(x=True, y=True)
        self.p.setTitle("ROV Mission Displayer", color="k", size="20px")

        self.p.addItem(self.vehicle)
        self.p.addItem(self.graph_item)
        self.i=0
        
        ############### Cage ###############
        # Cage
        s1 = pg.ScatterPlotItem(size=50, pen=pg.mkPen(None), symbol='s',brush=pg.mkBrush(255, 0, 0, 200))
        spots =[{'pos': [0,0]}]
        s1.addPoints(spots)
        self.p.addItem(s1)

        # Line
        curve = pg.PlotCurveItem(pen=({'color': '#f12828', 'width': 3}), skipFiniteCheck=True)
        self.p.addItem(curve)
        curve.setData(x=[0,1000], y=[0,0])
        ############### Cage ###############

        
        self.timer = QtCore.QTimer()
        self.timer.setInterval(20)
        self.timer.timeout.connect(self.update_plot_data)
        self.timer.start()
        self.show()

    
    def update_plot_data(self):
        if self.usbl_data!=None and self.heading!=None:
            self.vehicle.setPos(self.usbl_data.position.x,self.usbl_data.position.y)
            self.vehicle.setStyle(angle=self.heading)
            self.i+=1
            # self.graph_item.setData(x=self.x, y=self.y*np.sin(0.01*self.i),color='#288af1')  # Update the data.
            # self.graph_item.setData(x=[0,0.5,1], y=[0,.5,1],brush='#288af1')  # Update the data.
        else:
            self.vehicle.setPos(0,0)
            self.vehicle.setStyle(angle=0)

def ros_callback(data):
    global main
    try :
        main.usbl_data=data
    except:
        pass

def ros_compass(data):
    global main
    try :
        main.heading=data.data
    except:
        pass

################################## ROS ##################################
rospy.init_node('usbl_data_viewer', anonymous=True)
rospy.Subscriber("/Usbl_test", Usbl, ros_callback)
rospy.Subscriber("/mavros/global_position/compass_hdg", Float64, ros_compass)
ros_thread = threading.Thread(target=rospy.spin)
ros_thread.start()
################################## ROS ##################################


################################## Pyqt ##################################
app = QtWidgets.QApplication(sys.argv)
main = MainWindow()
sys.exit(app.exec_())
################################## Pyqt ##################################


