import cv2
import numpy as np

N_LINESAVER_BUFFER = 5
IMG_PATH = '/home/kite9240/catautonomous_ws/src/autonomous2018-ssu/vision/cal_distance/src/right_2_5m.png'

mtx_ls = [630.071853, 0, 322.309971, 0, 632.842228, 247.329905, 0, 0, 1] #camera matrix
dist = np.array([0.010162, -0.060262, 0.001452, -0.001965, 0]) #distortion matrix
fx_=1 #resize x 계수
fy_=1 #resize y 계수

CAMERA_NAME = 'right_'

class LineSaver():
    MAX_SLOPE = 9999
    v_or_s = 0.3
    def debugPrint(self):
        print('------debugPrint---------')
        print('len of recent, mathistory : ',len(self.recent), len(self.matHistory))
        print('state of vlines: ', self.vlines)
        print('state of hlines: ', self.hlines)
        print('-------------------------')

    def __init__(self, n_history):
        self.vlines = [] # vertical
        self.hlines = [] # horizontal

        #history management memberlineSaver.debugPrint()
        self.matHistory = []
        self.recent = []
        self.n_history = n_history
    def add(self, first, second):
        #first, second = (x1, y1), (x2, y2)

        #calf line equation
        if (first[0] - second[0]) == 0:
            slope = LineSaver.MAX_SLOPE #assign max slope
            intercept = first[0]#when slope is in infinite state, intercept indicates x's intercept
        else :
            slope = (first[1] - second[1]) / (first[0]-second[0])
            intercept = first[1] - int(slope * first[0])

        if abs(slope) >  LineSaver.v_or_s:
            self.vlines.append((slope, intercept))
        else :
            self.hlines.append((slope, intercept))

        #save log
        self.recent.append((slope, intercept))
        if len(self.recent) > self.n_history:
            self.recent.pop(0)

    def draw(self, mat):
        #check valid call
        if len(self.recent) <= 0:
            return

        #save log
        self.matHistory.append(mat.copy())
        del mat
        if len(self.matHistory) > self.n_history:
            self.matHistory.pop(0) #erase [0]

        #draw line
        slope = self.recent[-1][0]
        intercept = self.recent[-1][1]

        start = (-100, int(-100 * slope + intercept))
        end = (2000, int(2000 * slope + intercept))
        if slope == LineSaver.MAX_SLOPE:
            start = (intercept, -10)#see upper why intercept is in x's space
            end = (intercept, 2000)

        return cv2.line(self.matHistory[-1].copy(), start, end, (255, 0, 0), 1)
    def undo(self):
        if len(self.matHistory) <= 0:
            raise RuntimeError("do not click undo too much! I failed to handle it...")
        #remove from list
        if len(self.vlines) > 0 and self.vlines[-1][1] == self.recent[-1][1]:
            self.vlines.pop(-1)
        elif len(self.hlines) > 0 and self.hlines[-1][1] == self.recent[-1][1]:
            self.hlines.pop(-1)
        else :
            raise RuntimeError('why control reaches here....')

        #remove recent log
        self.recent.pop(-1)

        #process mat history
        mat = self.matHistory[-1].copy()
        self.matHistory.pop(-1)

        print('success undo!')

        return mat
    def save(self, mat):
        global CAMERA_NAME

        PATH = '../data/'+CAMERA_NAME+'calibration.txt'
        PATH2 = '../data/'+CAMERA_NAME+'calibration.png'
        PATH3 = '../data/'+CAMERA_NAME+'calibrationLine.txt'

        #sort hlines
        self.hlines.sort(key=lambda k:k[1])
	    #sort vlines
        self.vlines.sort(key=lambda k:k[0])
        middle_idx = 0
        if abs(self.vlines[0][0]) < abs(self.vlines[-1][0]):
            middle_idx = len(self.vlines) - 1
        middle = self.vlines[middle_idx]
        if middle_idx == 0:
            temp = self.vlines[0]
            for n in range(0,len(self.vlines)//2):
                print('n :',n)
                self.vlines[n] = self.vlines[n + 1]
        elif middle_idx == len(self.vlines) - 1:
            temp = self.vlines[-1]
            n = len(self.vlines) - 1
            while n > len(self.vlines)//2:
                print('n :',n)
                self.vlines[n] = self.vlines[n-1]
                n = n - 1

            # for n in range(len(self.vlines), len(self.vlines)//2):
            #     print('n :',n)
            #     self.vlines[n] = self.vlines[n-1]
        else :
            print("why control reaches here...")
        self.vlines[0], self.vlines[1] = self.vlines[1], self.vlines[0]
        self.vlines[3], self.vlines[4] = self.vlines[4], self.vlines[3]
        self.vlines[len(self.vlines)//2] = middle


        print('hlines : ', self.hlines)
        print('vlines : ', self.vlines)

        #write to the file
        f = open(PATH,'w')
        coorbuf = []
        linebuf = ''
        for hline in self.hlines:
            for vline in self.vlines:
                if vline[0] == LineSaver.MAX_SLOPE:
                    x = vline[1]
                    y = hline[0] * x + hline[1]
                elif hline[0] == 0:
                    x = (hline[1] - vline[1]) / vline[0]
                    y = hline[1]
                else :
                    x = -(hline[1] - vline[1]) / (hline[0] - vline[0])
                    y = (hline[1] / hline[0] - vline[1] / vline[0]) / (1 / hline[0] - 1 / vline[0])
                coorbuf.append((x,y))

            #sort corebuf by x
            coorbuf.sort(key=lambda k: k[0])
            for coor in coorbuf:
                linebuf += str(int(coor[0])) + ',' + str(int(coor[1])) + ','
            linebuf = linebuf[:-1]
            f.write(linebuf + '\n')
            coorbuf = []
            linebuf = ''
        f.close()
        print('save file : ', PATH)
        cv2.imwrite(PATH2, mat)
        print('save img : ', PATH2)

        #save slope info
        f3 = open(PATH3, 'w')
        linebuf = ''
        for hline in self.hlines:
            if hline[0] != 0:
                linebuf += str(hline[0]) + ',' + str(hline[1])
            else :
                linebuf += '0,' + str(hline[1])
            linebuf += ','
        linebuf = linebuf[:-1]
        linebuf += '\n'
        f3.write(linebuf)

        linebuf = ''
        for vline in self.vlines:
            if vline[0] != LineSaver.MAX_SLOPE:
                linebuf += str(vline[0]) + ',' + str(vline[1])
            else:
                linebuf += '9999,' + str(vline[1])
            linebuf += ','
        linebuf=linebuf[:-1]
        linebuf += '\n'
        f3.write(linebuf)
        print('save file2 : ', PATH3)


X_LINE_MAX = 300

X_ADD_LINE = 0
Y_ADD_LINE = 50
X_UNDO_LINE = 0
Y_UNDO_LINE = 100
X_SAVE_LINE = 0
Y_SAVE_LINE = 150

coor = []
lineSaver = LineSaver(N_LINESAVER_BUFFER)
drawFlag = False
undoFlag = False
saveFlag = False
def onButtonClicked(event, x, y, flags, param):
    global drawFlag
    global undoFlag
    global saveFlag
    global coor

    if event == cv2.EVENT_LBUTTONDOWN:

        #erase line?
        if (x < X_LINE_MAX) and (Y_ADD_LINE <= y <= Y_UNDO_LINE):
            undoFlag = True
            print("try to undo...")
            return
        #or save line?
        elif (x < X_LINE_MAX) and (Y_UNDO_LINE <= y <= Y_SAVE_LINE):
            saveFlag = True
            return
        #or just save the coordinate?
        coor.append((x, y))
        print("add coordinate : ", coor)

        # save line?
        # if (x < X_LINE_MAX) and (y < Y_ADD_LINE):
        if len(coor) >= 2:
            lineSaver.add(coor[0], coor[1])
            coor = []
            drawFlag = True
            print("Draw a line...")
            return
        #    return

    elif event == cv2.EVENT_LBUTTONDBLCLK:
        coor.clear()
        print("clear coordinate : ", coor)

if __name__=='__main__':
    mtx = np.zeros((3,3), np.float64)
    mtx_ls = [708.611809, 0, 320.331083, 0, 703.012319, 260.343059, 0, 0, 1]
    for i in range(0, 3):
        for j in range(0, 3):
            mtx[i][j] = mtx_ls[3*i + j]
    img = cv2.imread(IMG_PATH)
    img = cv2.undistort(img, mtx, dist)
    img = cv2.resize(img, None, fx=fx_, fy=fy_)

    cv2.rectangle(img, (0,0), (300, Y_ADD_LINE), (0, 0, 255), cv2.FILLED)
    cv2.putText(img, 'ADD LINE', (X_ADD_LINE,Y_ADD_LINE), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,0), 5)
    cv2.rectangle(img, (0, Y_ADD_LINE + 5), (300, Y_UNDO_LINE), (255, 0, 0), cv2.FILLED)
    cv2.putText(img, 'UNDO LINE', (X_UNDO_LINE, Y_UNDO_LINE), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 5)
    cv2.rectangle(img, (0, Y_UNDO_LINE + 5), (300, Y_SAVE_LINE), (0, 255, 0), cv2.FILLED)
    cv2.putText(img, 'SAVE LINE', (X_SAVE_LINE, Y_SAVE_LINE), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 5)

    cv2.namedWindow('img')
    cv2.setMouseCallback('img', onButtonClicked)

    while (1):
        cv2.imshow('img', img)
        if cv2.waitKey(100) & 0xFF == 27:
            break
        if drawFlag == True:
            img = lineSaver.draw(img)
            lineSaver.debugPrint()
            drawFlag = False
        if undoFlag == True:
            try:
                img = lineSaver.undo()
            except RuntimeError:
                print("buffer is empty!")
            lineSaver.debugPrint()
            undoFlag = False
        if saveFlag == True:
            lineSaver.save(img)
            break
    cv2.destroyAllWindows()
