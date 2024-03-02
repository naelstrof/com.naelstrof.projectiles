using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Assertions;

namespace Polynomials {
    
public static class RootSolver {
    private static PolynomialCubic cubic = new PolynomialCubic();
    private static PolynomialQuartic quartic = new PolynomialQuartic();

    private const float maxZeroDistanceFromPeakOrValley = 20f;
    // ax⁴+bx³+cx²+dx+e = 0
    public static void GetRootsForQuartic(List<float> roots, float a, float b, float c, float d, float e, float tolerance = 0.025f, int maxIterations = 12) {
        if (a == 0f) {
            GetRootsForCubic(roots, b, c, d, e, tolerance, maxIterations);
            return;
        }

        roots.Clear();
        GetRootsForCubic(roots, a * 4f, b * 3f, c*2f, d, tolerance, maxIterations);
        if (roots.Count == 0) {
            return;
        }
        roots.Sort();
        roots.Insert(0, roots[0] - maxZeroDistanceFromPeakOrValley);
        roots.Add(roots[^1] + maxZeroDistanceFromPeakOrValley);
        int count = roots.Count;
        quartic.SetConstants(a, b, c, d, e);
        for (int i = 0; i < count-1; i++) {
            GetRootBinarySearch(roots, quartic.Evaluate, roots[i], roots[i + 1], tolerance, maxIterations);
        }
        for (int i = 0; i < count; i++) {
            roots.RemoveAt(0);
        }
    }

    // ax³+bx²+cx+d = 0
    public static void GetRootsForCubic(List<float> roots, float a, float b, float c, float d, float tolerance = 0.025f, int maxIterations = 12) {
        if (a == 0f) {
            GetRootsForQuadratic(roots, b, c, d);
            return;
        }

        roots.Clear();
        GetRootsForQuadratic(roots, a * 3f, b * 2f, c);
        if (roots.Count == 0) {
            return;
        }
        roots.Sort();
        roots.Insert(0, roots[0] - maxZeroDistanceFromPeakOrValley);
        roots.Add(roots[^1] + maxZeroDistanceFromPeakOrValley);
        int count = roots.Count;
        cubic.SetConstants(a, b, c, d);
        for (int i = 0; i < count-1; i++) {
            GetRootBinarySearch(roots, cubic.Evaluate, roots[i], roots[i + 1]);
        }
        for (int i = 0; i < count; i++) {
            roots.RemoveAt(0);
        }
    }

    // ax²+bx+c = 0
    public static void GetRootsForQuadratic(IList<float> roots, float a, float b, float c) {
        if (a == 0) {
            GetRootForLinear(roots, b, c);
            return;
        }
        
        float discriminant = b * b - 4f * a * c;
        // Only non-real solutions.
        if (discriminant < 0f) {
            return;
        }

        roots.Add((-b + Mathf.Sqrt(discriminant)) / (2f * a));
        roots.Add((-b - Mathf.Sqrt(discriminant)) / (2f * a));
    }

    // ax+b=0
    public static void GetRootForLinear(IList<float> roots, float a, float b) {
        // Infinitely many solutions, uh oh! have one
        if (b == 0f && a == 0f) {
            roots.Add(0);
            return;
        }
        // No solution
        if (a == 0f) {
            return;
        }
        roots.Add(-b / a);
    }

    private delegate float PolynomialFunction(float t);
    private static void GetRootBinarySearch(IList<float> roots, PolynomialFunction function, float ta, float tb, float tolerance = 0.025f, int maxIterations = 12) {
        float fa = function.Invoke(ta);
        float fb = function.Invoke(tb);
        // No solutions (must be a local peak/valley), or just isn't valid
        if (fa * fb > 0) {
            return;
        }

        float t = ta;
        for (int i = 0; i < maxIterations; i++) {
            t = Mathf.Lerp(ta, tb, 0.5f);
            float ft = function.Invoke(t);
            if (ft == 0 || Mathf.Abs(tb - ta) / 2f < tolerance) {
                roots.Add(t);
                return;
            }
            if (Mathf.Approximately(Mathf.Sign(ft),Mathf.Sign(function.Invoke(ta)))) {
                ta = t;
            } else {
                tb = t;
            }
        }
        // Hit max iterations, this is our best guess.
        roots.Add(t);
    }
}

}
