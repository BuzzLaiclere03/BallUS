# This Python file uses the following encoding: utf-8
import sys
sys.path.append("/usr/lib/python3/dist-packages")
import libcamera
import numpy as np
import matplotlib.pyplot as plt
import serial
import cv2
import time
from ultralytics import YOLO

from PySide6.QtWidgets import QApplication, QMainWindow, QWidget,QGraphicsDropShadowEffect, QSlider,QListWidget,QListWidgetItem, QComboBox, QScrollArea, QLabel, QVBoxLayout, QPushButton, QLineEdit
from PySide6.QtGui import QImage, QPixmap, QPainter, QPolygon, QBrush, QColor, QFontDatabase, QFont
from PySide6.QtCore import QTimer, Qt, QPoint, QSize, QThread, Signal
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from threading import Thread
from picamera2 import Picamera2,Preview
from datetime import datetime

#variable global
SIZE_X = 1030
SIZE_Y = 560
ActiveLine = None
IsDrawing = False
SelectedFocus = 1
theta_yaw = 0
theta_pitch = 0
Conf = 0.4
isRecording = False
Mode3Active = False
takePicture = False
recordTime = 0
#dictionnaire des different objet possiible pour le suivi avvec camera
dictFocus = {0: "person",
1: "bicycle",
2: "car",
3: "motorcycle",
4: "airplane",
5: "bus",
6: "train",
7: "truck",
8: "boat",
9: "traffic light",
10: "fire hydrant",
11: "stop sign",
12: "parking meter",
13: "bench",
14: "bird",
15: "cat",
16: "dog",
17: "horse",
18: "sheep",
19: "cow",
20: "elephant",
21: "bear",
22: "zebra",
23: "giraffe",
24: "backpack",
25: "umbrella",
26: "handbag",
27: "tie",
28: "suitcase",
29: "frisbee",
30: "skis",
31: "snowboard",
32: "sports ball",
33: "kite",
34: "baseball bat",
35: "baseball glove",
36: "skateboard",
37: "surfboard",
38: "tennis racket",
39: "bottle",
40: "wine glass",
41: "cup",
42: "fork",
43: "knife",
44: "spoon",
45: "bowl",
46: "banana",
47: "apple",
48: "sandwich",
49: "orange",
50: "broccoli",
51: "carrot",
52: "hot dog",
53: "pizza",
54: "donut",
55: "cake",
56: "chair",
57: "couch",
58: "potted plant",
59: "bed",
60: "dining table",
61: "toilet",
62: "tv",
63: "laptop",
64: "mouse",
65: "remote",
66: "keyboard",
67: "cell phone",
68: "microwave",
69: "oven",
70: "toaster",
71: "sink",
72: "refrigerator",
73: "book",
74: "clock",
75: "vase",
76: "scissors",
77: "teddy bear",
78: "hair drier",
79: "toothbrush"}

#dictionnaire des listes HMI ,, option de configuration
dictOption = {1:"Mode 1", 2:"Mode 2", 3:"Mode 3", 4:"Mode 4",5:"Mode 5"}
dictSource = {1:"Motor Angles",2:"Follow Objets Angles"}

#modele pour l analyse du suivi camera
model = YOLO("yolov8n_ncnn_model", task="detect")

class PictureButtom(QPushButton):  #classe du bouton pour faiire une photo
    def __init__(self, parent = None):
        super().__init__("Picture",parent)
        self.setFixedSize(QSize(150,150))
        self.setStyleSheet(f"""
        QPushButton{{
        border-radius: 75px;
        background-color: #abcdff;
        font-size:20px;}}""")
        self.move(800,275)
        self.clicked.connect(self.ButtomPressed)
    def ButtomPressed(self):
        global takePicture
        takePicture = True       #indique le bouton photo a ete presse
class RecordButtom(QPushButton): #classe du bouton pour faire une video
    def __init__(self, parent = None):
        super().__init__("Record",parent)
        self.setFixedSize(QSize(150,150))
        self.setStyleSheet(f"""
        QPushButton{{
        border-radius: 75px;
        background-color: #90EE90;
        font-size:20px;}}""")
        self.move(100,275)
        self.clicked.connect(self.ButtomPressed)

    def ButtomPressed(self):
        global isRecording
        if (isRecording):  #regarde si un enregiistrement etait en court et ajuste en fonction
            self.setText("Record")
            self.setStyleSheet("""
            QPushButton {
                border-radius: 75px;
                background-color: #90EE90;
                font-size: 20px;
            }
            """)
            isRecording = False  #acti
        else:
            self.setText("Stop")
            self.setStyleSheet("""
            QPushButton {
                border-radius: 75px;
                background-color: red;
                font-size: 20px;
            }
            """)
            isRecording = True

class CameraWidget(QWidget):       #fenetre camera
    def __init__(self, parent = None):
        super().__init__(parent)
        self.setFixedSize(1680,1050)
        self.image_label = QLabel(self)
        self.image_label.setFixedSize(1680,1050)
        self.image_label.setAlignment(Qt.AlignCenter)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.updateFrame)
        #initialisation camera
        self.picam2 = Picamera2()
        config = self.picam2.create_preview_configuration(main={"format": 'RGB888', "size": (1280, 1280)})
        self.picam2.configure(config)
        self.picam2.start()
        self.timer.start(30)#30ms
        self.searchTime = 0
        self.hasSearched = False
        self.wasRecording = False

    def updateFrame(self):
        new_size = self.size()
        #capture image
        frame = self.picam2.capture_array()
        flipped_frame = cv2.flip(frame, 0)
        global takePicture
        if takePicture:  #regarde si il y a une requete de photo
            takePicture = False
            cv2.imwrite(f"{datetime.now()}.jpg", flipped_frame)
        global isRecording
        global recordTime
        if isRecording:   #regarde si il y a une requete de video
            if(not self.wasRecording): #si aucun enregistrement etait en cours, on creer lee fiichier d enregistrment
                fourcc = cv2.VideoWriter_fourcc(*'MP4V')  # ou 'MJPG', 'MP4V' selon le format
                self.out = cv2.VideoWriter(f"{datetime.now()}.mp4", fourcc, 20, (1280, 1280))
                self.startRecord = time.time()
            self.out.write(flipped_frame) #on ajoute une frame
            self.wasRecording = True
            recordTime = time.time()-self.startRecord
        else:
            if self.wasRecording:   #si un eenregistrement etait en court on ferme le fichier
                self.out.release()
                self.out = None
                self.wasRecording = False

        global Mode3Active
        if Mode3Active: #si mode 3 selectione, faire analyse pour suivi
            flipped_frame = self.analyse(flipped_frame)
        # Conversion de l'image OpenCV (ndarray) en QImage
        rgb_image = cv2.cvtColor(flipped_frame, cv2.COLOR_BGR2RGB)  # Conversion BGR->RGB
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)

        # Créer un QPixmap à partir de l'image Qt
        pixmap = QPixmap.fromImage(qt_image)

        # Redimensionner l'image pour correspondre à la taille du widget, en gardant le ratio
        self.image_label.setPixmap(pixmap.scaled(new_size, Qt.KeepAspectRatio, Qt.SmoothTransformation))


    def analyse(self,flipped_frame):
        frame_start_time = time.time()  # Début du traitement de la frame

        # Run YOLO model on the captured frame and store the results
        results = model.predict(flipped_frame, imgsz = 320)
        detections = results[0]

        x_center = []
        y_center = []

        for det in detections.boxes:
            global Conf
            x, y, w, h = det.xywh[0]  # Coordonnées du centre et dimensions
            conf = det.conf[0].item()  # Confiance
            cls = int(det.cls[0].item())  # Classe détectée
            global SelectedFocus
            if conf > Conf and cls == SelectedFocus:
                self.searchTime = 0
                self.hasSearched =False
                x, y, w, h = map(int, [x, y, w, h])
                cv2.rectangle(flipped_frame, (x - w // 2, y - h // 2),
                                (x + w // 2, y + h // 2), (0, 255, 0), 2)
                cv2.putText(flipped_frame, f"{cls} {conf:.2f}", (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                x_center.append(x)
                y_center.append(y)

                cv2.circle(flipped_frame, (x, y), 5, (0, 0, 255), -1)

            else :
                if not(self.hasSearched):
                    frame_end_time = time.time()
                    frame_time = frame_end_time - frame_start_time
                    self.searchTime += frame_time
                    if self.searchTime > 0.5:
                        self.hasSearched = True
                        self.get_rotation_angles(640, 640)
                return flipped_frame

            if x_center:
                x_center_moy = int(np.mean(x_center))
                y_center_moy = int(np.mean(y_center))


                self.get_rotation_angles(x_center_moy, y_center_moy)


                cv2.circle(flipped_frame, (x_center_moy, y_center_moy), 5, (0, 0, 255), -1)

        # Output the visual detection data, we will draw this on our camera preview window
        #annotated_frame = detections[0].plot()

        # Calculer la vitesse de la frame
        frame_end_time = time.time()
        frame_time = frame_end_time - frame_start_time
        fps = 1 / frame_time if frame_time > 0 else 0

        # Afficher les FPS sur l'image
        cv2.putText(flipped_frame, f"FPS: {fps:.1f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)

        return flipped_frame

    def get_rotation_angles(self,x_center, y_center):
        """
        Calcule les angles de rotation nécessaires pour suivre une cible détectée.
        """
        FOV_x = 66  # FOV horizontal en degrés
        FOV_y = 41 #FOV vertical en degrés
        #current_size = self.size()
        width = 1280
        height = 1280
        x_img = width / 2
        y_img = height / 2

        delta_x = x_center - x_img
        delta_y = y_center - y_img

        global theta_yaw
        global theta_pitch
        theta_yaw = (delta_x / width) * FOV_x
        theta_pitch = (delta_y / height) * FOV_y

class DynamicPLot(FigureCanvas): #classe pour graphe
    def __init__(self, x, y, w, h, parent = None):
        self.n = 1
        self.fig = Figure()
        self.ax = self.fig.add_subplot(111)
        super().__init__(self.fig)
        self.setParent(parent)
        self.x_data = list(range(10))
        self.y_data = [1,2,3,4,5,6,7,8,9,0]
        self.line, = self.ax.plot(self.x_data, self.y_data, 'r-')
        self.setGeometry(x,y,w,h)

    def updatePLot(self): #fonction exemple, manquue de temps d<integrer avec le reste
        self.y_data.pop(0)
        self.y_data.append(self.n)
        if self.n == 9:
            self.n = 0
        else :
            self.n += 1
        self.line.set_ydata(self.y_data)

        self.draw()

class SerialReader(Thread): #classe pour communication seeriel
    def __init__(self, port, input, updateDataLabel, baud_rate=9600):
        super().__init__()
        try:
            self.serial_port = serial.Serial(port, baud_rate)
            self.serial_port.timeout = 1
            self.running = True
            self.input = input
            self.updateDataLabel = updateDataLabel
        except:
            self.serial_port = None
            self.running = False

    def run(self):
        while self.running:
            if self.serial_port.in_waiting > 0:
                data = self.serial_port.readline().decode('utf-8').strip()
                value = ""
                i = 0
                for caracter in data:
                    if caracter != ',':
                        value += caracter
                    else:
                        self.input[i] = value
                        value = ""
                        i+=1
                self.input[i] = value
                self.received_data = data
                self.updateDataLabel()
            else:
                self.received_data = None

    def stop(self):
        self.running = False
        self.serial_port.close()

    def send_data(self, message):
        if self.serial_port != None:
            if self.serial_port.is_open:
                self.serial_port.write(message.encode('utf-8'))

class CustomComboBox(QComboBox): #classe pour les liste en fonction des dictionnaire
    def __init__(self, dict, x, y, change, parent=None):
        super().__init__(parent)
        liste = []
        for key in dict.keys():
            liste.append(dict[key])
        self.addItems(liste)
        self.move(x,y)

rounded_shadow_style = """  #style pour les interfaces
    QPushButton {
        border-radius: 20px;
        background-color: #f0f0f0;
        color: black;
        font-weight: bold;
        padding: 10px;
        border: 1px solid gray;
    }
    QPushButton:hover {
        background-color: #e0e0e0;
    }
"""

def apply_shadow(widget):
    shadow = QGraphicsDropShadowEffect()
    shadow.setBlurRadius(15)
    shadow.setOffset(3, 3)
    shadow.setColor(QColor(0, 0, 0, 160))
    widget.setGraphicsEffect(shadow)



class key_keyboard(QPushButton):   #objet touche de clavier
    def __init__(self, input, pos, cacher, parent=None):
            self.input = input
            super().__init__(input, parent)
            self.move((pos-1)*(SIZE_X//13), SIZE_Y-100)
            self.setFixedSize(SIZE_X//13, 100)

            # Style arrondi avec ombre
            self.setStyleSheet("""
                QPushButton {
                    border-radius: 25px;
                    background-color: white;
                    color: black;
                    font-size: 14px;
                    border: 2px solid #aaa;
                }
                QPushButton:hover {
                    background-color: #ddd;
                }
            """)

            # Appliquer l’ombree
            apply_shadow(self)

            self.cacher = cacher
            self.clicked.connect(self.write)
            self.timer = QTimer(self)
    def write(self):
        global ActiveLine  #recuperee le champ actif pour y ecrir
        text = ActiveLine.text()
        if (self.input == "backspace" and sys.getsizeof(text) > 0):
            text_list = list(text)
            text_list.pop()
            ActiveLine.setText(''.join(text_list))
        elif(self.input == "return"):
            self.cacher()
            ActiveLine.clearFocus()
            ActiveLine = None
        else:
            ActiveLine.setText(ActiveLine.text()+self.input)

class CustomLineEdit(QLineEdit): #classe champ d ecriirture specialiser pouur les differrent evenements
    def __init__(self, montrer, parent = None):
        super().__init__(parent)
        self.montrer = montrer

    def focusInEvent(self, event):  #evenement d entree
        super().focusInEvent(event)
        global ActiveLine
        ActiveLine = self
        self.montrer()

    def focusOutEvent(self, event): #evenement de sortie
        super().focusOutEvent(event)


class TabButton(QPushButton):   #bouton personnalisee pour les pages
    def __init__(self, text, color,id,changement, parent=None):
        super().__init__(parent)
        self.setFixedSize(100, 25)
        self.setAttribute(Qt.WidgetAttribute.WA_TranslucentBackground)
        self.text = text
        self.color = color
        self.id = id
        self.changement = changement
        self.setCursor(Qt.CursorShape.PointingHandCursor)
        apply_shadow(self)


    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        painter.setBrush(QBrush(QColor(self.color)))

        points = QPolygon([
            QPoint(0, 0),
            QPoint(100, 0),
            QPoint(100, 25),
            QPoint(25, 25)
        ])
        painter.drawPolygon(points)

        text_rect = self.rect().adjusted(25, 0, 0, 0)
        painter.setPen(Qt.GlobalColor.black)
        painter.drawText(text_rect, Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter, self.text)

        painter.end()

    def mousePressEvent(self, event):
        super().mousePressEvent(event)
        self.changement(self.id)
class ControlPanel(QWidget): #page du panneau de control
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedSize(SIZE_X, SIZE_Y)
        self.setStyleSheet("background-color : white;")
        #$creation des differents ensembles d<objets utiles pour afficher et cacher dees groupes d objets
        self.keyboardBoxxLayout = QVBoxLayout()
        self.Tabs = QVBoxLayout()
        self.Page1 = QVBoxLayout()
        self.Page2 = QVBoxLayout()
        self.Page3 = QVBoxLayout()
        self.dictPage = {}
        self.dictPage[1] = self.Page1
        self.dictPage[2] = self.Page2
        self.actualPage = 1
        self.serialEntry = [None,None,None]

        #initialisation des diiffrereents interfaces
        self.ManualWrittingField()
        self.DataLabel()
        self.option()
        self.keyboard()
        self.Tab()
        self.Graph()
        self.Communication()
        self.CameraFunction()

    #fonction appele au 100ms
    def cycle(self):
        self.graph.updatePLot()
        self.sendData()
        self.Chrono()
        self.LabelFollowObjectAngles.setText(f"Y : {theta_yaw}\nP : {theta_pitch}")

    def DataLabel(self): #creations des interfaces d affichage de certaines donnees
        self.BorderDataLabel = QLabel(self)
        self.BorderDataLabel.move(425,75)
        self.BorderDataLabel.setFixedSize(325,150)
        self.BorderDataLabel.setStyleSheet("""
            background-color: #abcdff;
            border: 2px solid black;
            padding: 10px;
            border-radius: 20px;
        """)


        self.Page1.addWidget(self.BorderDataLabel)
        self.TitleDataLabel = QLabel("Motor angle           Turn cam",self)
        self.TitleDataLabel.move(450,50)
        self.TitleDataLabel.setStyleSheet("""
            font-family: Arial;
            font-size: 20px;
            font-weight: bold;
            line-height: 20px;  /* Ajuste l'interligne */
            color: blue;
            background-color: white;""")
        self.LabelMotoAngles = QLabel(f"Y : {self.serialEntry[0]}\nP : {self.serialEntry[1]}\nR : {self.serialEntry[2]}", self)
        self.Page1.addWidget(self.TitleDataLabel)
        self.LabelMotoAngles.move(450,100)
        self.LabelMotoAngles.setFixedSize(100, 100)
        self.LabelMotoAngles.setStyleSheet("""
            font-family: Arial;
            font-size: 20px;
            font-weight: bold;
            line-height: 20px;  /* Ajuste l'interligne */
            color: black;
            background-color: #abcdff;""")
        self.Page1.addWidget(self.LabelMotoAngles)
        global theta_yaw
        global theta_pitch
        self.LabelFollowObjectAngles = QLabel(f"Y : 0\nP: 0", self)
        self.LabelFollowObjectAngles.setFixedSize(100, 100)
        self.LabelFollowObjectAngles.move(625,100)
        self.LabelFollowObjectAngles.setStyleSheet("""
            font-family: Arial;
            font-size: 20px;
            font-weight: bold;
            line-height: 20px;  /* Ajuste l'interligne */
            color: black;
            background-color: #abcdff;""")
        self.Page1.addWidget(self.LabelFollowObjectAngles)


    def ManualWrittingField(self): #creation de l interface d ecriiture pour le mode manuel
        self.BorderManualWrittingField = QLabel(self)
        self.BorderManualWrittingField.move(25,75)
        self.BorderManualWrittingField.setFixedSize(325,150)
        self.BorderManualWrittingField.setStyleSheet("""
            background-color: #abcdff;
            border: 2px solid black;
            padding: 10px;
            border-radius: 20px;
        """)
        self.Page1.addWidget(self.BorderManualWrittingField)
        self.TitleManualWrittingField = QLabel("Angular Control",self)
        self.TitleManualWrittingField.move(50,50)
        self.TitleManualWrittingField.setStyleSheet("""
            font-family: Arial;
            font-size: 20px;
            font-weight: bold;
            line-height: 20px;  /* Ajuste l'interligne */
            color: blue;
            background-color: white;""")
        self.Page1.addWidget(self.TitleManualWrittingField)
        self.x = 0
        self.y = 0
        self.z = 0
        self.LabelManualWrittingField = QLabel(f"Y : {self.x} \nP : {self.y}\nR : {self.z}", self)
        self.LabelManualWrittingField.setStyleSheet("""
            font-family: Arial;
            font-size: 20px;
            font-weight: bold;
            line-height: 20px;  /* Ajuste l'interligne */
            color: black;
            background-color: #abcdff;""")
        self.LabelManualWrittingField.setFixedSize(100, 100)
        self.LabelManualWrittingField.move(50,85)
        self.Page1.addWidget(self.LabelManualWrittingField)
        self.input_fieldy = CustomLineEdit(self.showKeyboard, self)
        self.input_fieldy.move(200,100)
        self.input_fieldy.returnPressed.connect(self.on_text_change)
        self.Page1.addWidget(self.input_fieldy)
        self.input_fieldp = CustomLineEdit(self.showKeyboard, self)
        self.input_fieldp.move(200,125)
        self.input_fieldp.returnPressed.connect(self.on_text_change)
        self.Page1.addWidget(self.input_fieldp)
        self.input_fieldr = CustomLineEdit(self.showKeyboard, self)
        self.input_fieldr.move(200,150)
        self.input_fieldr.returnPressed.connect(self.hideKeyboard)
        self.Page1.addWidget(self.input_fieldr)

    def option(self): #creation des interface de configuation de lla gimball(mode, focus, facteur de confiance)
        self.focus = CustomComboBox(dictFocus, 800, 125, self.sendData,self)
        self.mode = CustomComboBox(dictOption, 800, 200,self.sendData,self)
        self.TitleFocusCamera = QLabel("Focus Camera",self)
        self.TitleFocusCamera.move(800,100)
        self.TitleFocusCamera.setStyleSheet("""
            font-family: Arial;
            font-size: 20px;
            font-weight: bold;
            line-height: 20px;  /* Ajuste l'interligne */
            color: blue;
            background-color: white;""")
        self.TitleControlMode = QLabel("Control Mode",self)
        self.TitleControlMode.move(800,175)
        self.TitleControlMode.setStyleSheet("""
            font-family: Arial;
            font-size: 20px;
            font-weight: bold;
            line-height: 20px;  /* Ajuste l'interligne */
            color: blue;
            background-color: white;""")
        self.Page1.addWidget(self.focus)
        self.Page1.addWidget(self.mode)
        self.Page1.addWidget(self.TitleFocusCamera)
        self.Page1.addWidget(self.TitleControlMode)
        self.TrustFactor()


    def Graph(self): #creation des interphacesla page2 pour le graphiques
        self.graph = DynamicPLot(25, 25, 880,470, self)
        self.graph.hide()
        self.Page2.addWidget(self.graph)
        self.source = CustomComboBox(dictSource, 850, 150, None, self)
        self.source.hide()
        self.Page2.addWidget(self.source)
    def Communication(self): #creation du module de communication entre le ardiuino et raspberry pi
        self.serial_reader = SerialReader('/dev/ttyACM0', self.serialEntry, self.updateDataLabel, 9600)
        self.serial_reader.start()
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.cycle)
        self.timer.start(100)
    def CameraFunction(self): #creation du module de camera
        self.Record = RecordButtom(self)
        self.Page1.addWidget(self.Record)
        self.Picture = PictureButtom(self)
        self.Page1.addWidget(self.Picture)
        self.chronoLabel = QLabel("--:--:---",self)
        self.chronoLabel.move(400, 300)
        self.chronoLabel.setStyleSheet("""
        font-family: Arial;
        font-size: 60px;
        font-weight: bold;
        line-height: 20px;  /* Ajuste l'interligne */
        color: black;
        background-color: white;""")
        self.chronoLabel.setFixedSize(400, 100)
        self.Page1.addWidget(self.chronoLabel)
    def updateManualWrittingField(self):#mis a jour de l interface du manuel apres confirmation d entree
        self.LabelManualWrittingField.setText(f"Y : {self.x}\nP : {self.y}\nR : {self.z}")
        self.sendData()


    def sendData(self): #creer le message envoie les donne a l arduino
        mode = next((k for k, v in dictOption.items() if v == self.mode.currentText()),1)
        global SelectedFocus
        global theta_pitch
        global theta_yaw
        global Mode3Active
        if(mode == 3):
            Mode3Active = True
        else :
            Mode3Active = False
        SelectedFocus = next((k for k, v in dictFocus.items() if v == self.focus.currentText()),1)
        message = f"{mode},{self.x},{self.y},{self.z},{theta_yaw},{theta_pitch},0.00\n"
        self.serial_reader.send_data(message)
    def check(self, input):#traitement des valleurs entrees pour eviter les erreurs
        try:
            valeur = float(input)
            if valeur < -180:
                return 0
            elif valeur > 180:
                return 180
            else:
                return valeur
        except ValueError:
            return 0

    def on_text_change(self): #conmserve les valeurs d entree
        self.x = self.check(self.input_fieldy.text())
        self.y = self.check(self.input_fieldp.text())
        self.z = self.check(self.input_fieldr.text())
        self.updateManualWrittingField()
        self.hideKeyboard()

    def showKeyboard(self): #afficher les touches du clavier
        for i in range (self.keyboardBoxxLayout.count()):
            self.keyboardBoxxLayout.itemAt(i).widget().show()

    def hideKeyboard(self): #cacher les touches du claviier
        for i in range (self.keyboardBoxxLayout.count()):
            self.keyboardBoxxLayout.itemAt(i).widget().hide()

    def keyboard(self): #creation des touches clavier
        self.key_1 = key_keyboard("1", 1, self.hideKeyboard,self)
        self.key_2 = key_keyboard("2", 2, self.hideKeyboard,self)
        self.key_3 = key_keyboard("3", 3, self.hideKeyboard,self)
        self.key_4 = key_keyboard("4", 4, self.hideKeyboard,self)
        self.key_5 = key_keyboard("5", 5, self.hideKeyboard,self)
        self.key_6 = key_keyboard("6", 6, self.hideKeyboard,self)
        self.key_7 = key_keyboard("7", 7, self.hideKeyboard,self)
        self.key_8 = key_keyboard("8", 8, self.hideKeyboard,self)
        self.key_9 = key_keyboard("9", 9, self.hideKeyboard,self)
        self.key_0 = key_keyboard("0", 10, self.hideKeyboard,self)
        self.key_f = key_keyboard(".", 11, self.hideKeyboard,self)
        self.key_b = key_keyboard("backspace", 12, self.hideKeyboard,self)
        self.key_e = key_keyboard("return", 13, self.on_text_change,self)
        self.keyboardBoxxLayout.addWidget(self.key_1)
        self.keyboardBoxxLayout.addWidget(self.key_2)
        self.keyboardBoxxLayout.addWidget(self.key_3)
        self.keyboardBoxxLayout.addWidget(self.key_4)
        self.keyboardBoxxLayout.addWidget(self.key_5)
        self.keyboardBoxxLayout.addWidget(self.key_6)
        self.keyboardBoxxLayout.addWidget(self.key_7)
        self.keyboardBoxxLayout.addWidget(self.key_8)
        self.keyboardBoxxLayout.addWidget(self.key_9)
        self.keyboardBoxxLayout.addWidget(self.key_0)
        self.keyboardBoxxLayout.addWidget(self.key_f)
        self.keyboardBoxxLayout.addWidget(self.key_b)
        self.keyboardBoxxLayout.addWidget(self.key_e)






    def Tab(self):#creation des onglets pour les pages
        self.Tab1 = self.CreateTab("Tab1", 1, "#abcdff")
        self.Tab2 = self.CreateTab("Tab2", 2, "#abcdff")
        self.Tabs.addWidget(self.Tab1)
        self.Tabs.addWidget(self.Tab2)

    def CreateTab(self,t,n,c):#creations des objets onglets
        Tab = TabButton(t,c,n,self.changePage,self)
        Tab.setStyleSheet(f"color: black; font-size: 16px; text-align: center;")
        Tab.move(SIZE_X-100,(n-1)*25)
        return Tab

    def changePage(self, id): #change de pages
        if(id!=1):
            self.hideKeyboard()#cache le clavier si la page souhaite n<est pas lla premere
        for key in self.dictPage.keys():
            if key == id:
                for element in range(self.dictPage[key].count()):
                    self.dictPage[key].itemAt(element).widget().show() #affiche la page recherchee
            elif key == self.actualPage:
                for element in range(self.dictPage[key].count()):
                    self.dictPage[key].itemAt(element).widget().hide() #cache les pages non recherchee
        self.actualPage = id

    def closeEvent(self, event):
        self.serial_reader.stop()
        event.accept()

    def updateDataLabel(self):#mise a jours des interfaces d'affichage a chaque reception de message
        global theta_yaw
        global theta_pitch
        self.LabelMotoAngles.setText(f"Y : {self.serialEntry[0]}\nP : {self.serialEntry[1]}\nR : {self.serialEntry[2]}")


    def TrustFactor(self): #creation du potentiometre lineairee pour la configuration du facteur de conhfiancee
        self.TrustFactorSlider = QSlider(Qt.Vertical, self)
        self.TrustFactorSlider.resize(50,100)
        self.TrustFactorSlider.setRange(0, 100)
        self.TrustFactorSlider.setValue(40)
        self.TrustFactorSlider.setTickPosition(QSlider.TicksBelow)
        self.TrustFactorSlider.setTickInterval(1)
        self.TrustFactorSlider.move( 950, 100)
        self.TrustFactorSlider.show()
        self.TrustFactorLabel = QLabel("40",self)
        self.TrustFactorLabel.move(950,210)
        self.TrustFactorSlider.valueChanged.connect(self.updateTrustFactor)
        self.Page1.addWidget(self.TrustFactorSlider)
        self.Page1.addWidget(self.TrustFactorLabel)

    def updateTrustFactor(self):#mise a jour du facteur de confiance au cchangement d etat du potentiometre lineeaire
        global Conf
        self.TrustFactorLabel.setText(f"{self.TrustFactorSlider.value()}")
        Conf = self.TrustFactorSlider.value()/100

    def calculTime(self):#convversion de s en m:s:ms
        global recordTime
        total_seconds = recordTime
        m = int(total_seconds // 60)
        s = int(total_seconds % 60)
        ms = int((total_seconds - int(total_seconds)) * 1000)
        return m, s, ms

    def Chrono(self): #affichage du chronon d enregistrement
        global isRecording
        if isRecording:
            m,s,ms = self.calculTime()
            self.chronoLabel.setText(f"{m}:{s}:{ms}")
        else:
            self.chronoLabel.setText("--:--:---")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    widget1 = ControlPanel()
    widget2 = CameraWidget()
    widget1.show()
    widget2.show()

    sys.exit(app.exec())
