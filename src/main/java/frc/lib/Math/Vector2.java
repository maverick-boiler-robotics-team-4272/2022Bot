package frc.lib.Math;

public class Vector2 {
    public double x;
    public double y;

    public Vector2(double x, double y){
        this.x = x;
        this.y = y;
    }

    public Vector2 set(double x, double y){
        this.x = x;
        this.y = y;
        return this;
    }

    public Vector2 copy(Vector2 v){
        this.x = v.x;
        this.y = v.y;
        return this;
    }

    public Vector2 add(Vector2 v){
        this.x += v.x;
        this.y += v.y;
        return this;
    }

    public Vector2 sub(Vector2 v){
        this.x -= v.x;
        this.y -= v.y;
        return this;
    }

    public String toString(){
        return "{x:"+this.x;
    }
}
