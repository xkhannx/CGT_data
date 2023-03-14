function B = linResample(A, fOld, fNew)

t = (0:round(length(A) / fOld * fNew))/fNew;
x = 0:(1/fOld):((length(A)-1)/fOld);

B = interp1(x, A, t);