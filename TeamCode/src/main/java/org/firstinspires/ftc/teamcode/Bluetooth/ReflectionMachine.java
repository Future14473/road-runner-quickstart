package org.firstinspires.ftc.teamcode.Bluetooth;

import java.lang.reflect.Field;
import java.lang.reflect.Method;

public class ReflectionMachine {
    static Object getByString(Object actor, String name){
        Field f = null;
        try {
            f = actor.getClass().getField(name);
        } catch (NoSuchFieldException e) {
        }

        Method m = null;
        for(Method p:actor.getClass().getMethods()){
            if(p.getName().equals(name)){
                m = p;
                break;
            }
        }

        if(m != null) return m;
        if(f != null) return f;
        return null;
    }

    public static String methods(Object target){
        StringBuilder ret = new StringBuilder();
        Method[] l = target.getClass().getMethods();
        for(Method f:l){
            ret.append("Method: " + f + "\n");
        }
        return ret.append("\n").toString();
    }

    public static String fields(Object target){
        StringBuilder ret = new StringBuilder();
        Field[] l = target.getClass().getFields();
        for(Field f:l){
            ret.append(f + "\n");
        }
        return ret.append("\n").toString();
    }
}
