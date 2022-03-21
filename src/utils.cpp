

/**
 * Find the Greatest Common Denominator for two numbers
 */
int gcd(int x, int y)
{
    if (x == 0)
        return y;
    return gcd(y % x, x);
}