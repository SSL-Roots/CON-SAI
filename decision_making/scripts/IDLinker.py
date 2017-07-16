from GlobalData import GlobalInfo

def updateGlobalFriendIDs(IDList):
    GlobalInfo.friendIDs = list(IDList)

    count = 0

    listSize = len(IDList)
    for count in xrange(6):
        # if count < len(IDList):
        #     GlobalInfo.controls[count].set_id(IDList[count])
        # else:
        #     GlobalInfo.controls[count].delete_id()

        if count < listSize: 
            GlobalInfo.controls[count].set_id(IDList[listSize -1 - count])
        else:
            GlobalInfo.controls[count].delete_id()

def updateGlobalEnemyIDs(IDList):
    GlobalInfo.enemyIDs = list(IDList)
    
