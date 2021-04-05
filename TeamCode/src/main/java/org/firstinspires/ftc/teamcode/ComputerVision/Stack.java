package org.firstinspires.ftc.teamcode.ComputerVision;

import org.opencv.core.Rect;
import java.util.ArrayList;

public class Stack implements Comparable<Stack>{
    public int count = 0;
    public double avgHeight = 0;
    public Rect fullStack = new Rect();

    public Stack(Rect firstObj){
        this.count = 1;
        this.avgHeight = firstObj.height;
        this.fullStack = firstObj;
    }

    public int centerY(){
        return fullStack.y + (int)(fullStack.height/2.0);
    }

    public int centerX(){
        return fullStack.x + (int)(fullStack.width/2.0);
    }

    public void addExisting(Rect rect){
        count++;
        fullStack.x = (rect.x + (count-1)*fullStack.x)/(count);
        fullStack.width = (rect.width + (count-1)*fullStack.width)/(count);
        int maxY = Math.max(rect.y + rect.height, fullStack.y + fullStack.height);
        fullStack.y = Math.min(rect.y, fullStack.y);
        fullStack.height = maxY - fullStack.y;
        avgHeight = (rect.height + (count-1)*avgHeight)/(count);
    }

    public static int closeIn(ArrayList<Stack> list, Rect rect, double epsilonW, double epsilonH){
        for(int i = 0; i< list.size(); i++){
            if(Math.abs(rect.y - list.get(i).centerY()) < epsilonH/2 &&
                    Math.abs(rect.x - list.get(i).fullStack.x) < epsilonW/2){
                return -2;
            }
            if(Math.abs(list.get(i).fullStack.x - rect.x) < epsilonW &&
                    Math.abs(list.get(i).fullStack.width - rect.width) < epsilonW &&
                    Math.abs(list.get(i).avgHeight - rect.height) < epsilonH){
                return i;
            }
        }
        return -1;
    }

    @Override
    public int compareTo(Stack o){
        if(this.count > o.count){
            return 1;
        }
        else if(this.count < o.count){
            return -1;
        }
        else{
            return 0;
        }
    }

    //Returns the stack that's closest in terms of the y pixel value


    public static Rect closestStack(ArrayList<Stack> list){
        int maxY = 0;
        int index = 0;
        for(int i = 0; i< list.size(); i++){
            Rect currRect = list.get(i).fullStack;
            if(maxY < currRect.y + currRect.height){
                maxY = currRect.y + currRect.height;
                index = i;
            }
        }
        return list.get(index).fullStack;
    }

}
