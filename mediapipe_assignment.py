import mediapipe as mp
import cv2
import random
mp_hands=mp.solutions.hands
mpDraw = mp.solutions.drawing_utils

def cond_rock(ip,it,mp,mt,rp,rt,pp,pt):
    if(ip<it) and (mp<mt) and (rp<rt) and (pp <pt):
        return True
    return False
def cond_paper(ip,it,mp,mt,rp,rt,pp,pt):
    if(ip>it) and (mp>mt) and (rp>rt) and (pp>pt):
        return True
    return False
def cond_scissor(ip,it,mp,mt,rp,rt,pp,pt):
    if(ip>it) and (mp>mt) and (rp<rt) and (pp <pt):
        return True
    return False
cap=cv2.VideoCapture(0)
with mp_hands.Hands(static_image_mode=True,max_num_hands=1,min_detection_confidence=0.5)as hands:   
    while(cap.isOpened()):
        ret,frame = cap.read()
        if ret == False:
            break
        results= hands.process(cv2.cvtColor(frame,cv2.COLOR_BGR2RGB))   
        if results.multi_hand_landmarks:
            for handLms in results.multi_hand_landmarks:
                mpDraw.draw_landmarks(frame, handLms,mp_hands.HAND_CONNECTIONS)
            hn = results.multi_hand_landmarks[0]
            ip = hn.landmark[6].y
            it = hn.landmark[8].y
            mp = hn.landmark[10].y
            mt = hn.landmark[12].y
            rp = hn.landmark[14].y
            rt = hn.landmark[16].y
            pp = hn.landmark[18].y
            pt = hn.landmark[20].y
            txt = ''
            if(cond_rock(ip,it,mp,mt,rp,rt,pp,pt)):
                txt="rock"
                sc=1
            elif(cond_paper(ip,it,mp,mt,rp,rt,pp,pt)):
                txt="paper"
                sc=2   
            elif(cond_scissor(ip,it,mp,mt,rp,rt,pp,pt)):
                txt="scissor"
                sc=3
            comp_image = cv2.putText(img=frame,text ="Computer",org =(15,35),fontFace = cv2.FONT_HERSHEY_DUPLEX,fontScale=1.5,color=(125,246,55),thickness =3)
            n = random.randint(1,3)
            if (n==1):
                image = cv2.putText(img=frame,text ="Rock",org =(15,75),fontFace = cv2.FONT_HERSHEY_DUPLEX,fontScale=1.5,color=(125,246,55),thickness =3)
            elif (n==2):
                image = cv2.putText(img=frame,text ="Paper",org =(15,75),fontFace = cv2.FONT_HERSHEY_DUPLEX,fontScale=1.5,color=(125,246,55),thickness =3)
            elif (n==3):
                image = cv2.putText(img=frame,text ="Scissor",org =(15,75),fontFace = cv2.FONT_HERSHEY_DUPLEX,fontScale=1.5,color=(125,246,55),thickness =3)
            if(((n==1)and(sc==2)) or (n==2)and(sc==3)) or ((n==3)and(sc==1)):
                winner = cv2.putText(img=frame,text ="You win",org =(200,350),fontFace = cv2.FONT_HERSHEY_DUPLEX,fontScale=1.5,color=(125,246,55),thickness =3)
            elif(n==sc):
                winner = cv2.putText(img=frame,text ="Draw",org =(200,350),fontFace = cv2.FONT_HERSHEY_DUPLEX,fontScale=1.5,color=(125,246,55),thickness =3)
            else:
                winner = cv2.putText(img=frame,text ="Computer wins",org =(200,350),fontFace = cv2.FONT_HERSHEY_DUPLEX,fontScale=1.5,color=(125,246,55),thickness =3)
            new_image = cv2.putText(img=frame,text =txt,org =(200,200),fontFace = cv2.FONT_HERSHEY_DUPLEX,fontScale=3.0,color=(125,246,55),thickness =3)

            cv2.imshow("fin",frame)
            if cv2.waitKey(3) & 0xFF == ord('q'):
                break

cap.release()
cv2.destroyAllWindows()
