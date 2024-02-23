using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Polynomials {
public abstract class Polynomial {
    public abstract float Evaluate(float t);
}

public class PolynomialQuartic : Polynomial {
    private float a, b, c, d, e;
    public void SetConstants(float a, float b, float c, float d, float e) {
        this.a = a; this.b = b; this.c = c; this.d = d; this.e = e;
    }
    public override float Evaluate(float t) {
        return a * (t * t * t * t) +
               b * (t * t * t) +
               c * (t * t) +
               d * t +
               e;
    }
}

public class PolynomialCubic : Polynomial {
    private float a, b, c, d;
    public void SetConstants(float a, float b, float c, float d) {
        this.a = a; this.b = b; this.c = c; this.d = d;
    }
    public override float Evaluate(float t) {
        return a * (t * t * t) +
               b * (t * t) +
               c * t +
               d;
    }
}

public class PolynomialQuadratic : Polynomial {
    private float a, b, c;
    public void SetConstants(float a, float b, float c) {
        this.a = a; this.b = b; this.c = c;
    }
    public override float Evaluate(float t) {
        return a * (t * t) +
               b * t +
               c;
    }
}

public class PolynomialLinear : Polynomial {
    private float a, b;
    public void SetConstants(float a, float b) {
        this.a = a; this.b = b;
    }
    public override float Evaluate(float t) {
        return a * t + b;
    }
}

}
