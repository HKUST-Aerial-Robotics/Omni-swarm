import matplotlib.pyplot as plt
import numpy as np



def expo(value, e):
	x = np.clip(value, - 1, 1)
	ec = np.clip(e, 0, 1)
	return (1 - ec) * x + ec * x * x * x


const T superexpo(const T &value, const T &e, const T &g)
{
	T x = constrain(value, (T) - 1, (T) 1);
	T gc = constrain(g, (T) 0, (T) 0.99);
	return expo(x, e) * (1 - gc) / (1 - fabsf(x) * gc);
}