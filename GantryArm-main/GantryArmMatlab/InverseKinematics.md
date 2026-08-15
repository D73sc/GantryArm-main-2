# Homogeneous Transformation Matrices from Base to Joint

## T_01

$$
\left(\begin{array}{cccc} \cos\left(\theta _{1}\right) & -\sin\left(\theta _{1}\right) & 0 & 90\\ \sin\left(\theta _{1}\right) & \cos\left(\theta _{1}\right) & 0 & 0\\ 0 & 0 & 1 & -\frac{591}{2}\\ 0 & 0 & 0 & 1 \end{array}\right)
$$

## T_10

$$
\left(\begin{array}{cccc} \cos\left(\theta _{1}\right) & -\sin\left(\theta _{1}\right) & 0 & 90\\ \sin\left(\theta _{1}\right) & \cos\left(\theta _{1}\right) & 0 & 0\\ 0 & 0 & 1 & -\frac{591}{2}\\ 0 & 0 & 0 & 1 \end{array}\right)
$$

## T_02

$$
\left(\begin{array}{cccc} -\cos\left(\theta _{1}\right)\,\sin\left(\theta _{2}\right) & -\cos\left(\theta _{1}\right)\,\cos\left(\theta _{2}\right) & \sin\left(\theta _{1}\right) & 200\,\cos\left(\theta _{1}\right)+90\\ -\sin\left(\theta _{1}\right)\,\sin\left(\theta _{2}\right) & -\cos\left(\theta _{2}\right)\,\sin\left(\theta _{1}\right) & -\cos\left(\theta _{1}\right) & 200\,\sin\left(\theta _{1}\right)\\ \cos\left(\theta _{2}\right) & -\sin\left(\theta _{2}\right) & 0 & -\frac{591}{2}\\ 0 & 0 & 0 & 1 \end{array}\right)
$$

## T_21

$$
\left(\begin{array}{cccc} -\sin\left(\theta _{2}\right) & -\cos\left(\theta _{2}\right) & 0 & 200\\ 0 & 0 & -1 & 0\\ \cos\left(\theta _{2}\right) & -\sin\left(\theta _{2}\right) & 0 & 0\\ 0 & 0 & 0 & 1 \end{array}\right)
$$

## T_03

$$
\left(\begin{array}{cccc} \sin\left(\theta _{1}\right)\,\sin\left(\theta _{3}\right)-\cos\left(\theta _{1}\right)\,\cos\left(\theta _{3}\right)\,\sin\left(\theta _{2}\right) & \cos\left(\theta _{3}\right)\,\sin\left(\theta _{1}\right)+\cos\left(\theta _{1}\right)\,\sin\left(\theta _{2}\right)\,\sin\left(\theta _{3}\right) & \cos\left(\theta _{1}\right)\,\cos\left(\theta _{2}\right) & 200\,\cos\left(\theta _{1}\right)+\frac{305\,\cos\left(\theta _{1}\right)\,\cos\left(\theta _{2}\right)}{2}+90\\ -\cos\left(\theta _{1}\right)\,\sin\left(\theta _{3}\right)-\cos\left(\theta _{3}\right)\,\sin\left(\theta _{1}\right)\,\sin\left(\theta _{2}\right) & \sin\left(\theta _{1}\right)\,\sin\left(\theta _{2}\right)\,\sin\left(\theta _{3}\right)-\cos\left(\theta _{1}\right)\,\cos\left(\theta _{3}\right) & \cos\left(\theta _{2}\right)\,\sin\left(\theta _{1}\right) & 200\,\sin\left(\theta _{1}\right)+\frac{305\,\cos\left(\theta _{2}\right)\,\sin\left(\theta _{1}\right)}{2}\\ \cos\left(\theta _{2}\right)\,\cos\left(\theta _{3}\right) & -\cos\left(\theta _{2}\right)\,\sin\left(\theta _{3}\right) & \sin\left(\theta _{2}\right) & \frac{305\,\sin\left(\theta _{2}\right)}{2}-\frac{591}{2}\\ 0 & 0 & 0 & 1 \end{array}\right)
$$

## T_32

$$
\left(\begin{array}{cccc} \cos\left(\theta _{3}\right) & -\sin\left(\theta _{3}\right) & 0 & 0\\ 0 & 0 & -1 & -\frac{305}{2}\\ \sin\left(\theta _{3}\right) & \cos\left(\theta _{3}\right) & 0 & 0\\ 0 & 0 & 0 & 1 \end{array}\right)
$$

## T_04

$$
\left(\begin{array}{cccc} \cos\left(\theta _{4}\right)\,\left(\sin\left(\theta _{1}\right)\,\sin\left(\theta _{3}\right)-\cos\left(\theta _{1}\right)\,\cos\left(\theta _{3}\right)\,\sin\left(\theta _{2}\right)\right)+\sin\left(\theta _{4}\right)\,\left(\cos\left(\theta _{3}\right)\,\sin\left(\theta _{1}\right)+\cos\left(\theta _{1}\right)\,\sin\left(\theta _{2}\right)\,\sin\left(\theta _{3}\right)\right) & \cos\left(\theta _{4}\right)\,\left(\cos\left(\theta _{3}\right)\,\sin\left(\theta _{1}\right)+\cos\left(\theta _{1}\right)\,\sin\left(\theta _{2}\right)\,\sin\left(\theta _{3}\right)\right)-\sin\left(\theta _{4}\right)\,\left(\sin\left(\theta _{1}\right)\,\sin\left(\theta _{3}\right)-\cos\left(\theta _{1}\right)\,\cos\left(\theta _{3}\right)\,\sin\left(\theta _{2}\right)\right) & \cos\left(\theta _{1}\right)\,\cos\left(\theta _{2}\right) & 200\,\cos\left(\theta _{1}\right)+381\,\cos\left(\theta _{1}\right)\,\cos\left(\theta _{2}\right)+90\\ -\cos\left(\theta _{4}\right)\,\left(\cos\left(\theta _{1}\right)\,\sin\left(\theta _{3}\right)+\cos\left(\theta _{3}\right)\,\sin\left(\theta _{1}\right)\,\sin\left(\theta _{2}\right)\right)-\sin\left(\theta _{4}\right)\,\left(\cos\left(\theta _{1}\right)\,\cos\left(\theta _{3}\right)-\sin\left(\theta _{1}\right)\,\sin\left(\theta _{2}\right)\,\sin\left(\theta _{3}\right)\right) & \sin\left(\theta _{4}\right)\,\left(\cos\left(\theta _{1}\right)\,\sin\left(\theta _{3}\right)+\cos\left(\theta _{3}\right)\,\sin\left(\theta _{1}\right)\,\sin\left(\theta _{2}\right)\right)-\cos\left(\theta _{4}\right)\,\left(\cos\left(\theta _{1}\right)\,\cos\left(\theta _{3}\right)-\sin\left(\theta _{1}\right)\,\sin\left(\theta _{2}\right)\,\sin\left(\theta _{3}\right)\right) & \cos\left(\theta _{2}\right)\,\sin\left(\theta _{1}\right) & 200\,\sin\left(\theta _{1}\right)+381\,\cos\left(\theta _{2}\right)\,\sin\left(\theta _{1}\right)\\ \cos\left(\theta _{2}\right)\,\cos\left(\theta _{3}\right)\,\cos\left(\theta _{4}\right)-\cos\left(\theta _{2}\right)\,\sin\left(\theta _{3}\right)\,\sin\left(\theta _{4}\right) & -\cos\left(\theta _{2}\right)\,\cos\left(\theta _{3}\right)\,\sin\left(\theta _{4}\right)-\cos\left(\theta _{2}\right)\,\cos\left(\theta _{4}\right)\,\sin\left(\theta _{3}\right) & \sin\left(\theta _{2}\right) & 381\,\sin\left(\theta _{2}\right)-\frac{591}{2}\\ 0 & 0 & 0 & 1 \end{array}\right)
$$

## T_43

$$
\left(\begin{array}{cccc} \cos\left(\theta _{4}\right) & -\sin\left(\theta _{4}\right) & 0 & 0\\ \sin\left(\theta _{4}\right) & \cos\left(\theta _{4}\right) & 0 & 0\\ 0 & 0 & 1 & \frac{457}{2}\\ 0 & 0 & 0 & 1 \end{array}\right)
$$

## T_05

$$
\left(\begin{array}{cccc} \cos\left(\theta _{4}\right)\,\left(\sin\left(\theta _{1}\right)\,\sin\left(\theta _{3}\right)-\cos\left(\theta _{1}\right)\,\cos\left(\theta _{3}\right)\,\sin\left(\theta _{2}\right)\right)+\sin\left(\theta _{4}\right)\,\left(\cos\left(\theta _{3}\right)\,\sin\left(\theta _{1}\right)+\cos\left(\theta _{1}\right)\,\sin\left(\theta _{2}\right)\,\sin\left(\theta _{3}\right)\right) & -\cos\left(\theta _{1}\right)\,\cos\left(\theta _{2}\right) & \cos\left(\theta _{4}\right)\,\left(\cos\left(\theta _{3}\right)\,\sin\left(\theta _{1}\right)+\cos\left(\theta _{1}\right)\,\sin\left(\theta _{2}\right)\,\sin\left(\theta _{3}\right)\right)-\sin\left(\theta _{4}\right)\,\left(\sin\left(\theta _{1}\right)\,\sin\left(\theta _{3}\right)-\cos\left(\theta _{1}\right)\,\cos\left(\theta _{3}\right)\,\sin\left(\theta _{2}\right)\right) & 200\,\cos\left(\theta _{1}\right)+381\,\cos\left(\theta _{1}\right)\,\cos\left(\theta _{2}\right)+118\,\cos\left(\theta _{4}\right)\,\left(\cos\left(\theta _{3}\right)\,\sin\left(\theta _{1}\right)+\cos\left(\theta _{1}\right)\,\sin\left(\theta _{2}\right)\,\sin\left(\theta _{3}\right)\right)-118\,\sin\left(\theta _{4}\right)\,\left(\sin\left(\theta _{1}\right)\,\sin\left(\theta _{3}\right)-\cos\left(\theta _{1}\right)\,\cos\left(\theta _{3}\right)\,\sin\left(\theta _{2}\right)\right)+90\\ -\cos\left(\theta _{4}\right)\,\left(\cos\left(\theta _{1}\right)\,\sin\left(\theta _{3}\right)+\cos\left(\theta _{3}\right)\,\sin\left(\theta _{1}\right)\,\sin\left(\theta _{2}\right)\right)-\sin\left(\theta _{4}\right)\,\left(\cos\left(\theta _{1}\right)\,\cos\left(\theta _{3}\right)-\sin\left(\theta _{1}\right)\,\sin\left(\theta _{2}\right)\,\sin\left(\theta _{3}\right)\right) & -\cos\left(\theta _{2}\right)\,\sin\left(\theta _{1}\right) & \sin\left(\theta _{4}\right)\,\left(\cos\left(\theta _{1}\right)\,\sin\left(\theta _{3}\right)+\cos\left(\theta _{3}\right)\,\sin\left(\theta _{1}\right)\,\sin\left(\theta _{2}\right)\right)-\cos\left(\theta _{4}\right)\,\left(\cos\left(\theta _{1}\right)\,\cos\left(\theta _{3}\right)-\sin\left(\theta _{1}\right)\,\sin\left(\theta _{2}\right)\,\sin\left(\theta _{3}\right)\right) & 200\,\sin\left(\theta _{1}\right)+381\,\cos\left(\theta _{2}\right)\,\sin\left(\theta _{1}\right)-118\,\cos\left(\theta _{4}\right)\,\left(\cos\left(\theta _{1}\right)\,\cos\left(\theta _{3}\right)-\sin\left(\theta _{1}\right)\,\sin\left(\theta _{2}\right)\,\sin\left(\theta _{3}\right)\right)+118\,\sin\left(\theta _{4}\right)\,\left(\cos\left(\theta _{1}\right)\,\sin\left(\theta _{3}\right)+\cos\left(\theta _{3}\right)\,\sin\left(\theta _{1}\right)\,\sin\left(\theta _{2}\right)\right)\\ \cos\left(\theta _{2}\right)\,\cos\left(\theta _{3}\right)\,\cos\left(\theta _{4}\right)-\cos\left(\theta _{2}\right)\,\sin\left(\theta _{3}\right)\,\sin\left(\theta _{4}\right) & -\sin\left(\theta _{2}\right) & -\cos\left(\theta _{2}\right)\,\cos\left(\theta _{3}\right)\,\sin\left(\theta _{4}\right)-\cos\left(\theta _{2}\right)\,\cos\left(\theta _{4}\right)\,\sin\left(\theta _{3}\right) & 381\,\sin\left(\theta _{2}\right)-118\,\cos\left(\theta _{2}\right)\,\cos\left(\theta _{3}\right)\,\sin\left(\theta _{4}\right)-118\,\cos\left(\theta _{2}\right)\,\cos\left(\theta _{4}\right)\,\sin\left(\theta _{3}\right)-\frac{591}{2}\\ 0 & 0 & 0 & 1 \end{array}\right)
$$

## T_54

$$
\left(\begin{array}{cccc} 1 & 0 & 0 & 0\\ 0 & 0 & 1 & 118\\ 0 & -1 & 0 & 0\\ 0 & 0 & 0 & 1 \end{array}\right)
$$

## T_06

$$
\left(\begin{array}{cccc} \cos\left(\theta _{4}\right)\,\left(\sin\left(\theta _{1}\right)\,\sin\left(\theta _{3}\right)-\cos\left(\theta _{1}\right)\,\cos\left(\theta _{3}\right)\,\sin\left(\theta _{2}\right)\right)+\sin\left(\theta _{4}\right)\,\left(\cos\left(\theta _{3}\right)\,\sin\left(\theta _{1}\right)+\cos\left(\theta _{1}\right)\,\sin\left(\theta _{2}\right)\,\sin\left(\theta _{3}\right)\right) & -\cos\left(\theta _{1}\right)\,\cos\left(\theta _{2}\right) & \cos\left(\theta _{4}\right)\,\left(\cos\left(\theta _{3}\right)\,\sin\left(\theta _{1}\right)+\cos\left(\theta _{1}\right)\,\sin\left(\theta _{2}\right)\,\sin\left(\theta _{3}\right)\right)-\sin\left(\theta _{4}\right)\,\left(\sin\left(\theta _{1}\right)\,\sin\left(\theta _{3}\right)-\cos\left(\theta _{1}\right)\,\cos\left(\theta _{3}\right)\,\sin\left(\theta _{2}\right)\right) & 200\,\cos\left(\theta _{1}\right)+381\,\cos\left(\theta _{1}\right)\,\cos\left(\theta _{2}\right)+90\\ -\cos\left(\theta _{4}\right)\,\left(\cos\left(\theta _{1}\right)\,\sin\left(\theta _{3}\right)+\cos\left(\theta _{3}\right)\,\sin\left(\theta _{1}\right)\,\sin\left(\theta _{2}\right)\right)-\sin\left(\theta _{4}\right)\,\left(\cos\left(\theta _{1}\right)\,\cos\left(\theta _{3}\right)-\sin\left(\theta _{1}\right)\,\sin\left(\theta _{2}\right)\,\sin\left(\theta _{3}\right)\right) & -\cos\left(\theta _{2}\right)\,\sin\left(\theta _{1}\right) & \sin\left(\theta _{4}\right)\,\left(\cos\left(\theta _{1}\right)\,\sin\left(\theta _{3}\right)+\cos\left(\theta _{3}\right)\,\sin\left(\theta _{1}\right)\,\sin\left(\theta _{2}\right)\right)-\cos\left(\theta _{4}\right)\,\left(\cos\left(\theta _{1}\right)\,\cos\left(\theta _{3}\right)-\sin\left(\theta _{1}\right)\,\sin\left(\theta _{2}\right)\,\sin\left(\theta _{3}\right)\right) & 200\,\sin\left(\theta _{1}\right)+381\,\cos\left(\theta _{2}\right)\,\sin\left(\theta _{1}\right)\\ \cos\left(\theta _{2}\right)\,\cos\left(\theta _{3}\right)\,\cos\left(\theta _{4}\right)-\cos\left(\theta _{2}\right)\,\sin\left(\theta _{3}\right)\,\sin\left(\theta _{4}\right) & -\sin\left(\theta _{2}\right) & -\cos\left(\theta _{2}\right)\,\cos\left(\theta _{3}\right)\,\sin\left(\theta _{4}\right)-\cos\left(\theta _{2}\right)\,\cos\left(\theta _{4}\right)\,\sin\left(\theta _{3}\right) & 381\,\sin\left(\theta _{2}\right)-\frac{591}{2}\\ 0 & 0 & 0 & 1 \end{array}\right)
$$

## T_65

$$
\left(\begin{array}{cccc} 1 & 0 & 0 & 0\\ 0 & 1 & 0 & 0\\ 0 & 0 & 1 & -118\\ 0 & 0 & 0 & 1 \end{array}\right)
$$

## T_07

$$
\left(\begin{array}{cccc} \cos\left(\theta _{5}\right)\,\left(\cos\left(\theta _{4}\right)\,\left(\sin\left(\theta _{1}\right)\,\sin\left(\theta _{3}\right)-\cos\left(\theta _{1}\right)\,\cos\left(\theta _{3}\right)\,\sin\left(\theta _{2}\right)\right)+\sin\left(\theta _{4}\right)\,\left(\cos\left(\theta _{3}\right)\,\sin\left(\theta _{1}\right)+\cos\left(\theta _{1}\right)\,\sin\left(\theta _{2}\right)\,\sin\left(\theta _{3}\right)\right)\right)+\sin\left(\theta _{5}\right)\,\left(\cos\left(\theta _{4}\right)\,\left(\cos\left(\theta _{3}\right)\,\sin\left(\theta _{1}\right)+\cos\left(\theta _{1}\right)\,\sin\left(\theta _{2}\right)\,\sin\left(\theta _{3}\right)\right)-\sin\left(\theta _{4}\right)\,\left(\sin\left(\theta _{1}\right)\,\sin\left(\theta _{3}\right)-\cos\left(\theta _{1}\right)\,\cos\left(\theta _{3}\right)\,\sin\left(\theta _{2}\right)\right)\right) & \cos\left(\theta _{5}\right)\,\left(\cos\left(\theta _{4}\right)\,\left(\cos\left(\theta _{3}\right)\,\sin\left(\theta _{1}\right)+\cos\left(\theta _{1}\right)\,\sin\left(\theta _{2}\right)\,\sin\left(\theta _{3}\right)\right)-\sin\left(\theta _{4}\right)\,\left(\sin\left(\theta _{1}\right)\,\sin\left(\theta _{3}\right)-\cos\left(\theta _{1}\right)\,\cos\left(\theta _{3}\right)\,\sin\left(\theta _{2}\right)\right)\right)-\sin\left(\theta _{5}\right)\,\left(\cos\left(\theta _{4}\right)\,\left(\sin\left(\theta _{1}\right)\,\sin\left(\theta _{3}\right)-\cos\left(\theta _{1}\right)\,\cos\left(\theta _{3}\right)\,\sin\left(\theta _{2}\right)\right)+\sin\left(\theta _{4}\right)\,\left(\cos\left(\theta _{3}\right)\,\sin\left(\theta _{1}\right)+\cos\left(\theta _{1}\right)\,\sin\left(\theta _{2}\right)\,\sin\left(\theta _{3}\right)\right)\right) & \cos\left(\theta _{1}\right)\,\cos\left(\theta _{2}\right) & 200\,\cos\left(\theta _{1}\right)+671\,\cos\left(\theta _{1}\right)\,\cos\left(\theta _{2}\right)+90\\ -\cos\left(\theta _{5}\right)\,\left(\cos\left(\theta _{4}\right)\,\left(\cos\left(\theta _{1}\right)\,\sin\left(\theta _{3}\right)+\cos\left(\theta _{3}\right)\,\sin\left(\theta _{1}\right)\,\sin\left(\theta _{2}\right)\right)+\sin\left(\theta _{4}\right)\,\left(\cos\left(\theta _{1}\right)\,\cos\left(\theta _{3}\right)-\sin\left(\theta _{1}\right)\,\sin\left(\theta _{2}\right)\,\sin\left(\theta _{3}\right)\right)\right)-\sin\left(\theta _{5}\right)\,\left(\cos\left(\theta _{4}\right)\,\left(\cos\left(\theta _{1}\right)\,\cos\left(\theta _{3}\right)-\sin\left(\theta _{1}\right)\,\sin\left(\theta _{2}\right)\,\sin\left(\theta _{3}\right)\right)-\sin\left(\theta _{4}\right)\,\left(\cos\left(\theta _{1}\right)\,\sin\left(\theta _{3}\right)+\cos\left(\theta _{3}\right)\,\sin\left(\theta _{1}\right)\,\sin\left(\theta _{2}\right)\right)\right) & \sin\left(\theta _{5}\right)\,\left(\cos\left(\theta _{4}\right)\,\left(\cos\left(\theta _{1}\right)\,\sin\left(\theta _{3}\right)+\cos\left(\theta _{3}\right)\,\sin\left(\theta _{1}\right)\,\sin\left(\theta _{2}\right)\right)+\sin\left(\theta _{4}\right)\,\left(\cos\left(\theta _{1}\right)\,\cos\left(\theta _{3}\right)-\sin\left(\theta _{1}\right)\,\sin\left(\theta _{2}\right)\,\sin\left(\theta _{3}\right)\right)\right)-\cos\left(\theta _{5}\right)\,\left(\cos\left(\theta _{4}\right)\,\left(\cos\left(\theta _{1}\right)\,\cos\left(\theta _{3}\right)-\sin\left(\theta _{1}\right)\,\sin\left(\theta _{2}\right)\,\sin\left(\theta _{3}\right)\right)-\sin\left(\theta _{4}\right)\,\left(\cos\left(\theta _{1}\right)\,\sin\left(\theta _{3}\right)+\cos\left(\theta _{3}\right)\,\sin\left(\theta _{1}\right)\,\sin\left(\theta _{2}\right)\right)\right) & \cos\left(\theta _{2}\right)\,\sin\left(\theta _{1}\right) & 200\,\sin\left(\theta _{1}\right)+671\,\cos\left(\theta _{2}\right)\,\sin\left(\theta _{1}\right)\\ -\cos\left(\theta _{5}\right)\,\left(\cos\left(\theta _{2}\right)\,\sin\left(\theta _{3}\right)\,\sin\left(\theta _{4}\right)-\cos\left(\theta _{2}\right)\,\cos\left(\theta _{3}\right)\,\cos\left(\theta _{4}\right)\right)-\sin\left(\theta _{5}\right)\,\left(\cos\left(\theta _{2}\right)\,\cos\left(\theta _{3}\right)\,\sin\left(\theta _{4}\right)+\cos\left(\theta _{2}\right)\,\cos\left(\theta _{4}\right)\,\sin\left(\theta _{3}\right)\right) & \sin\left(\theta _{5}\right)\,\left(\cos\left(\theta _{2}\right)\,\sin\left(\theta _{3}\right)\,\sin\left(\theta _{4}\right)-\cos\left(\theta _{2}\right)\,\cos\left(\theta _{3}\right)\,\cos\left(\theta _{4}\right)\right)-\cos\left(\theta _{5}\right)\,\left(\cos\left(\theta _{2}\right)\,\cos\left(\theta _{3}\right)\,\sin\left(\theta _{4}\right)+\cos\left(\theta _{2}\right)\,\cos\left(\theta _{4}\right)\,\sin\left(\theta _{3}\right)\right) & \sin\left(\theta _{2}\right) & 671\,\sin\left(\theta _{2}\right)-\frac{591}{2}\\ 0 & 0 & 0 & 1 \end{array}\right)
$$

## T_76

$$
\left(\begin{array}{cccc} \cos\left(\theta _{5}\right) & -\sin\left(\theta _{5}\right) & 0 & 0\\ 0 & 0 & -1 & -290\\ \sin\left(\theta _{5}\right) & \cos\left(\theta _{5}\right) & 0 & 0\\ 0 & 0 & 0 & 1 \end{array}\right)
$$

