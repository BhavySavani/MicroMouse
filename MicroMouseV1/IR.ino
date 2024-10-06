//Author-Bhavy Savani And Ali


void walldetect(){
       if(current_direction==0){
            if(digitalRead(IRF)==LOW){bot_up=true;}
            if(digitalRead(IRLF)==LOW){bot_left=true;}
            if(digitalRead(IRRF)==LOW){bot_right=true;}
       } //IR Sensor Code Here
       if(current_direction==90){
            if(digitalRead(IRF)==LOW){bot_right=true;Serial.println("going here");}
            if(digitalRead(IRLF)==LOW){bot_up=true;}
            if(digitalRead(IRRF)==LOW){bot_down=true;}
       }
       if(current_direction==180){
            if(digitalRead(IRF)==LOW){bot_down=true;}
            if(digitalRead(IRLF)==LOW){bot_right=true;}
            if(digitalRead(IRRF)==LOW){bot_left=true;}
       }
       if(current_direction==270){
            if(digitalRead(IRF)==LOW){bot_left=true;}
            if(digitalRead(IRLF)==LOW){bot_down=true;}
            if(digitalRead(IRRF)==LOW){bot_up=true;}
       }
    }

void wall_update(int hwall[maze_size+1][maze_size],int vwall[maze_size][maze_size+1],int currentX,int currentY){
    bot_up=false,bot_left=false,bot_right=false,bot_down=false;
    walldetect();
    if(bot_up==true && hwall[currentX][currentY]==0 ){
        hwall[currentX][currentY]=hwall[currentX][currentY]+1;
    }
    if(bot_down==true && hwall[currentX+1][currentY]==0 ){
        hwall[currentX+1][currentY]=hwall[currentX+1][currentY]+1;
    }
    if(bot_left==true && vwall[currentX][currentY]==0){
        vwall[currentX][currentY]=vwall[currentX][currentY]+1;
    }
    if(bot_right==true && vwall[currentX][currentY+1]==0){
        vwall[currentX][currentY+1]=vwall[currentX][currentY+1]+1;
    }
}
