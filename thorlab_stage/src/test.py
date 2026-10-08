from tele_xy_thorlab_v5 import LinearStage


def main():
    Stage = LinearStage([0,1,0])
    #Stage.goHome()
    pos = Stage.getPos()
    #Stage.set_Velocity()
    print(pos)
    while(1):
        Stage.moveAbsolute([0,30,0])
        Stage.moveRelative([20,40,20])
    #Stage.moveAbsolute(50,50,50)
    pos = Stage.getPos()
    print(pos)
    pass

if __name__ =="__main__":
    main()




