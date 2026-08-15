# Homogeneous Transformation Matrices from Base to Joint

## T_21

$$
\left(\begin{array}{cccc} 0 & 1 & 0 & 0\\ 0 & 0 & -1 & -d_{1}\\ -1 & 0 & 0 & 0\\ 0 & 0 & 0 & 1 \end{array}\right)
$$

## T_02

$$
\left(\begin{array}{cccc} 0 & 1 & 0 & 0\\ 0 & 0 & -1 & -d_{1}\\ -1 & 0 & 0 & 0\\ 0 & 0 & 0 & 1 \end{array}\right)
$$

## T_32

$$
\left(\begin{array}{cccc} 0 & -1 & 0 & 0\\ 0 & 0 & -1 & -d_{2}\\ 1 & 0 & 0 & 0\\ 0 & 0 & 0 & 1 \end{array}\right)
$$

## T_03

$$
\left(\begin{array}{cccc} 0 & 0 & -1 & -d_{2}\\ -1 & 0 & 0 & -d_{1}\\ 0 & 1 & 0 & 0\\ 0 & 0 & 0 & 1 \end{array}\right)
$$

## T_43

$$
\left(\begin{array}{cccc} \cos\left(\theta _{3}\right) & -\sin\left(\theta _{3}\right) & 0 & 90\\ \sin\left(\theta _{3}\right) & \cos\left(\theta _{3}\right) & 0 & 0\\ 0 & 0 & 1 & -\frac{591}{2}\\ 0 & 0 & 0 & 1 \end{array}\right)
$$

## T_04

$$
\left(\begin{array}{cccc} 0 & 0 & -1 & \frac{591}{2}-d_{2}\\ -\cos\left(\theta _{3}\right) & \sin\left(\theta _{3}\right) & 0 & -d_{1}-90\\ \sin\left(\theta _{3}\right) & \cos\left(\theta _{3}\right) & 0 & 0\\ 0 & 0 & 0 & 1 \end{array}\right)
$$

## T_54

$$
\left(\begin{array}{cccc} -\sin\left(\theta _{4}\right) & -\cos\left(\theta _{4}\right) & 0 & 200\\ 0 & 0 & -1 & 0\\ \cos\left(\theta _{4}\right) & -\sin\left(\theta _{4}\right) & 0 & 0\\ 0 & 0 & 0 & 1 \end{array}\right)
$$

## T_05

$$
\left(\begin{array}{cccc} -\cos\left(\theta _{4}\right) & \sin\left(\theta _{4}\right) & 0 & \frac{591}{2}-d_{2}\\ \cos\left(\theta _{3}\right)\,\sin\left(\theta _{4}\right) & \cos\left(\theta _{3}\right)\,\cos\left(\theta _{4}\right) & -\sin\left(\theta _{3}\right) & -d_{1}-200\,\cos\left(\theta _{3}\right)-90\\ -\sin\left(\theta _{3}\right)\,\sin\left(\theta _{4}\right) & -\cos\left(\theta _{4}\right)\,\sin\left(\theta _{3}\right) & -\cos\left(\theta _{3}\right) & 200\,\sin\left(\theta _{3}\right)\\ 0 & 0 & 0 & 1 \end{array}\right)
$$

## T_65

$$
\left(\begin{array}{cccc} -\cos\left(\theta _{5}\right) & \sin\left(\theta _{5}\right) & 0 & 0\\ 0 & 0 & -1 & -381\\ -\sin\left(\theta _{5}\right) & -\cos\left(\theta _{5}\right) & 0 & 0\\ 0 & 0 & 0 & 1 \end{array}\right)
$$

## T_06

$$
\left(\begin{array}{cccc} \cos\left(\theta _{4}\right)\,\cos\left(\theta _{5}\right) & -\cos\left(\theta _{4}\right)\,\sin\left(\theta _{5}\right) & -\sin\left(\theta _{4}\right) & \frac{591}{2}-381\,\sin\left(\theta _{4}\right)-d_{2}\\ \sin\left(\theta _{3}\right)\,\sin\left(\theta _{5}\right)-\cos\left(\theta _{3}\right)\,\cos\left(\theta _{5}\right)\,\sin\left(\theta _{4}\right) & \cos\left(\theta _{5}\right)\,\sin\left(\theta _{3}\right)+\cos\left(\theta _{3}\right)\,\sin\left(\theta _{4}\right)\,\sin\left(\theta _{5}\right) & -\cos\left(\theta _{3}\right)\,\cos\left(\theta _{4}\right) & -d_{1}-200\,\cos\left(\theta _{3}\right)-381\,\cos\left(\theta _{3}\right)\,\cos\left(\theta _{4}\right)-90\\ \cos\left(\theta _{3}\right)\,\sin\left(\theta _{5}\right)+\cos\left(\theta _{5}\right)\,\sin\left(\theta _{3}\right)\,\sin\left(\theta _{4}\right) & \cos\left(\theta _{3}\right)\,\cos\left(\theta _{5}\right)-\sin\left(\theta _{3}\right)\,\sin\left(\theta _{4}\right)\,\sin\left(\theta _{5}\right) & \cos\left(\theta _{4}\right)\,\sin\left(\theta _{3}\right) & 200\,\sin\left(\theta _{3}\right)+381\,\cos\left(\theta _{4}\right)\,\sin\left(\theta _{3}\right)\\ 0 & 0 & 0 & 1 \end{array}\right)
$$

## T_76

$$
\left(\begin{array}{cccc} -\cos\left(\theta _{6}\right) & \sin\left(\theta _{6}\right) & 0 & 0\\ 0 & 0 & -1 & 0\\ -\sin\left(\theta _{6}\right) & -\cos\left(\theta _{6}\right) & 0 & 0\\ 0 & 0 & 0 & 1 \end{array}\right)
$$

## T_07

$$
\left(\begin{array}{cccc} \sin\left(\theta _{4}\right)\,\sin\left(\theta _{6}\right)-\cos\left(\theta _{4}\right)\,\cos\left(\theta _{5}\right)\,\cos\left(\theta _{6}\right) & \cos\left(\theta _{6}\right)\,\sin\left(\theta _{4}\right)+\cos\left(\theta _{4}\right)\,\cos\left(\theta _{5}\right)\,\sin\left(\theta _{6}\right) & \cos\left(\theta _{4}\right)\,\sin\left(\theta _{5}\right) & \frac{591}{2}-381\,\sin\left(\theta _{4}\right)-d_{2}\\ \cos\left(\theta _{3}\right)\,\cos\left(\theta _{4}\right)\,\sin\left(\theta _{6}\right)-\cos\left(\theta _{6}\right)\,\left(\sin\left(\theta _{3}\right)\,\sin\left(\theta _{5}\right)-\cos\left(\theta _{3}\right)\,\cos\left(\theta _{5}\right)\,\sin\left(\theta _{4}\right)\right) & \sin\left(\theta _{6}\right)\,\left(\sin\left(\theta _{3}\right)\,\sin\left(\theta _{5}\right)-\cos\left(\theta _{3}\right)\,\cos\left(\theta _{5}\right)\,\sin\left(\theta _{4}\right)\right)+\cos\left(\theta _{3}\right)\,\cos\left(\theta _{4}\right)\,\cos\left(\theta _{6}\right) & -\cos\left(\theta _{5}\right)\,\sin\left(\theta _{3}\right)-\cos\left(\theta _{3}\right)\,\sin\left(\theta _{4}\right)\,\sin\left(\theta _{5}\right) & -d_{1}-200\,\cos\left(\theta _{3}\right)-381\,\cos\left(\theta _{3}\right)\,\cos\left(\theta _{4}\right)-90\\ -\cos\left(\theta _{6}\right)\,\left(\cos\left(\theta _{3}\right)\,\sin\left(\theta _{5}\right)+\cos\left(\theta _{5}\right)\,\sin\left(\theta _{3}\right)\,\sin\left(\theta _{4}\right)\right)-\cos\left(\theta _{4}\right)\,\sin\left(\theta _{3}\right)\,\sin\left(\theta _{6}\right) & \sin\left(\theta _{6}\right)\,\left(\cos\left(\theta _{3}\right)\,\sin\left(\theta _{5}\right)+\cos\left(\theta _{5}\right)\,\sin\left(\theta _{3}\right)\,\sin\left(\theta _{4}\right)\right)-\cos\left(\theta _{4}\right)\,\cos\left(\theta _{6}\right)\,\sin\left(\theta _{3}\right) & \sin\left(\theta _{3}\right)\,\sin\left(\theta _{4}\right)\,\sin\left(\theta _{5}\right)-\cos\left(\theta _{3}\right)\,\cos\left(\theta _{5}\right) & 200\,\sin\left(\theta _{3}\right)+381\,\cos\left(\theta _{4}\right)\,\sin\left(\theta _{3}\right)\\ 0 & 0 & 0 & 1 \end{array}\right)
$$

## T_87

$$
\left(\begin{array}{cccc} -\cos\left(\theta _{7}\right) & \sin\left(\theta _{7}\right) & 0 & 0\\ 0 & 0 & -1 & -\mathrm{Load}-129\\ -\sin\left(\theta _{7}\right) & -\cos\left(\theta _{7}\right) & 0 & 0\\ 0 & 0 & 0 & 1 \end{array}\right)
$$

## T_08

$$
\left(\begin{array}{cccc} -\cos\left(\theta _{7}\right)\,\left(\sin\left(\theta _{4}\right)\,\sin\left(\theta _{6}\right)-\cos\left(\theta _{4}\right)\,\cos\left(\theta _{5}\right)\,\cos\left(\theta _{6}\right)\right)-\cos\left(\theta _{4}\right)\,\sin\left(\theta _{5}\right)\,\sin\left(\theta _{7}\right) & \sin\left(\theta _{7}\right)\,\left(\sin\left(\theta _{4}\right)\,\sin\left(\theta _{6}\right)-\cos\left(\theta _{4}\right)\,\cos\left(\theta _{5}\right)\,\cos\left(\theta _{6}\right)\right)-\cos\left(\theta _{4}\right)\,\cos\left(\theta _{7}\right)\,\sin\left(\theta _{5}\right) & -\cos\left(\theta _{6}\right)\,\sin\left(\theta _{4}\right)-\cos\left(\theta _{4}\right)\,\cos\left(\theta _{5}\right)\,\sin\left(\theta _{6}\right) & \frac{591}{2}-381\,\sin\left(\theta _{4}\right)-\left(\cos\left(\theta _{6}\right)\,\sin\left(\theta _{4}\right)+\cos\left(\theta _{4}\right)\,\cos\left(\theta _{5}\right)\,\sin\left(\theta _{6}\right)\right)\,\left(\mathrm{Load}+129\right)-d_{2}\\ \cos\left(\theta _{7}\right)\,\left(\cos\left(\theta _{6}\right)\,\left(\sin\left(\theta _{3}\right)\,\sin\left(\theta _{5}\right)-\cos\left(\theta _{3}\right)\,\cos\left(\theta _{5}\right)\,\sin\left(\theta _{4}\right)\right)-\cos\left(\theta _{3}\right)\,\cos\left(\theta _{4}\right)\,\sin\left(\theta _{6}\right)\right)+\sin\left(\theta _{7}\right)\,\left(\cos\left(\theta _{5}\right)\,\sin\left(\theta _{3}\right)+\cos\left(\theta _{3}\right)\,\sin\left(\theta _{4}\right)\,\sin\left(\theta _{5}\right)\right) & \cos\left(\theta _{7}\right)\,\left(\cos\left(\theta _{5}\right)\,\sin\left(\theta _{3}\right)+\cos\left(\theta _{3}\right)\,\sin\left(\theta _{4}\right)\,\sin\left(\theta _{5}\right)\right)-\sin\left(\theta _{7}\right)\,\left(\cos\left(\theta _{6}\right)\,\left(\sin\left(\theta _{3}\right)\,\sin\left(\theta _{5}\right)-\cos\left(\theta _{3}\right)\,\cos\left(\theta _{5}\right)\,\sin\left(\theta _{4}\right)\right)-\cos\left(\theta _{3}\right)\,\cos\left(\theta _{4}\right)\,\sin\left(\theta _{6}\right)\right) & -\sin\left(\theta _{6}\right)\,\left(\sin\left(\theta _{3}\right)\,\sin\left(\theta _{5}\right)-\cos\left(\theta _{3}\right)\,\cos\left(\theta _{5}\right)\,\sin\left(\theta _{4}\right)\right)-\cos\left(\theta _{3}\right)\,\cos\left(\theta _{4}\right)\,\cos\left(\theta _{6}\right) & -d_{1}-200\,\cos\left(\theta _{3}\right)-381\,\cos\left(\theta _{3}\right)\,\cos\left(\theta _{4}\right)-\left(\sin\left(\theta _{6}\right)\,\left(\sin\left(\theta _{3}\right)\,\sin\left(\theta _{5}\right)-\cos\left(\theta _{3}\right)\,\cos\left(\theta _{5}\right)\,\sin\left(\theta _{4}\right)\right)+\cos\left(\theta _{3}\right)\,\cos\left(\theta _{4}\right)\,\cos\left(\theta _{6}\right)\right)\,\left(\mathrm{Load}+129\right)-90\\ \cos\left(\theta _{7}\right)\,\left(\cos\left(\theta _{6}\right)\,\left(\cos\left(\theta _{3}\right)\,\sin\left(\theta _{5}\right)+\cos\left(\theta _{5}\right)\,\sin\left(\theta _{3}\right)\,\sin\left(\theta _{4}\right)\right)+\cos\left(\theta _{4}\right)\,\sin\left(\theta _{3}\right)\,\sin\left(\theta _{6}\right)\right)+\sin\left(\theta _{7}\right)\,\left(\cos\left(\theta _{3}\right)\,\cos\left(\theta _{5}\right)-\sin\left(\theta _{3}\right)\,\sin\left(\theta _{4}\right)\,\sin\left(\theta _{5}\right)\right) & \cos\left(\theta _{7}\right)\,\left(\cos\left(\theta _{3}\right)\,\cos\left(\theta _{5}\right)-\sin\left(\theta _{3}\right)\,\sin\left(\theta _{4}\right)\,\sin\left(\theta _{5}\right)\right)-\sin\left(\theta _{7}\right)\,\left(\cos\left(\theta _{6}\right)\,\left(\cos\left(\theta _{3}\right)\,\sin\left(\theta _{5}\right)+\cos\left(\theta _{5}\right)\,\sin\left(\theta _{3}\right)\,\sin\left(\theta _{4}\right)\right)+\cos\left(\theta _{4}\right)\,\sin\left(\theta _{3}\right)\,\sin\left(\theta _{6}\right)\right) & \cos\left(\theta _{4}\right)\,\cos\left(\theta _{6}\right)\,\sin\left(\theta _{3}\right)-\sin\left(\theta _{6}\right)\,\left(\cos\left(\theta _{3}\right)\,\sin\left(\theta _{5}\right)+\cos\left(\theta _{5}\right)\,\sin\left(\theta _{3}\right)\,\sin\left(\theta _{4}\right)\right) & 200\,\sin\left(\theta _{3}\right)+381\,\cos\left(\theta _{4}\right)\,\sin\left(\theta _{3}\right)-\left(\sin\left(\theta _{6}\right)\,\left(\cos\left(\theta _{3}\right)\,\sin\left(\theta _{5}\right)+\cos\left(\theta _{5}\right)\,\sin\left(\theta _{3}\right)\,\sin\left(\theta _{4}\right)\right)-\cos\left(\theta _{4}\right)\,\cos\left(\theta _{6}\right)\,\sin\left(\theta _{3}\right)\right)\,\left(\mathrm{Load}+129\right)\\ 0 & 0 & 0 & 1 \end{array}\right)
$$

## T_0_3_inv

$$
\left(\begin{array}{cccc} 0 & -1 & 0 & -d_{1}\\ 0 & 0 & 1 & 0\\ -1 & 0 & 0 & -d_{2}\\ 0 & 0 & 0 & 1 \end{array}\right)
$$

## T_3_5

$$
\left(\begin{array}{cccc} -\cos\left(\theta _{3}\right)\,\sin\left(\theta _{4}\right) & -\cos\left(\theta _{3}\right)\,\cos\left(\theta _{4}\right) & \sin\left(\theta _{3}\right) & 200\,\cos\left(\theta _{3}\right)+90\\ -\sin\left(\theta _{3}\right)\,\sin\left(\theta _{4}\right) & -\cos\left(\theta _{4}\right)\,\sin\left(\theta _{3}\right) & -\cos\left(\theta _{3}\right) & 200\,\sin\left(\theta _{3}\right)\\ \cos\left(\theta _{4}\right) & -\sin\left(\theta _{4}\right) & 0 & -\frac{591}{2}\\ 0 & 0 & 0 & 1 \end{array}\right)
$$

## T_5_8

$$
\left(\begin{array}{cccc} \sin\left(\theta _{5}\right)\,\sin\left(\theta _{7}\right)-\cos\left(\theta _{5}\right)\,\cos\left(\theta _{6}\right)\,\cos\left(\theta _{7}\right) & \cos\left(\theta _{7}\right)\,\sin\left(\theta _{5}\right)+\cos\left(\theta _{5}\right)\,\cos\left(\theta _{6}\right)\,\sin\left(\theta _{7}\right) & \cos\left(\theta _{5}\right)\,\sin\left(\theta _{6}\right) & \cos\left(\theta _{5}\right)\,\sin\left(\theta _{6}\right)\,\left(\mathrm{Load}+129\right)\\ -\cos\left(\theta _{7}\right)\,\sin\left(\theta _{6}\right) & \sin\left(\theta _{6}\right)\,\sin\left(\theta _{7}\right) & -\cos\left(\theta _{6}\right) & -\cos\left(\theta _{6}\right)\,\left(\mathrm{Load}+129\right)-381\\ -\cos\left(\theta _{5}\right)\,\sin\left(\theta _{7}\right)-\cos\left(\theta _{6}\right)\,\cos\left(\theta _{7}\right)\,\sin\left(\theta _{5}\right) & \cos\left(\theta _{6}\right)\,\sin\left(\theta _{5}\right)\,\sin\left(\theta _{7}\right)-\cos\left(\theta _{5}\right)\,\cos\left(\theta _{7}\right) & \sin\left(\theta _{5}\right)\,\sin\left(\theta _{6}\right) & \sin\left(\theta _{5}\right)\,\sin\left(\theta _{6}\right)\,\left(\mathrm{Load}+129\right)\\ 0 & 0 & 0 & 1 \end{array}\right)
$$

## T_3_8

$$
\left(\begin{array}{cccc} \cos\left(\theta _{3}\right)\,\cos\left(\theta _{4}\right)\,\cos\left(\theta _{7}\right)\,\sin\left(\theta _{6}\right)-\cos\left(\theta _{3}\right)\,\sin\left(\theta _{4}\right)\,\left(\sin\left(\theta _{5}\right)\,\sin\left(\theta _{7}\right)-\cos\left(\theta _{5}\right)\,\cos\left(\theta _{6}\right)\,\cos\left(\theta _{7}\right)\right)-\sin\left(\theta _{3}\right)\,\left(\cos\left(\theta _{5}\right)\,\sin\left(\theta _{7}\right)+\cos\left(\theta _{6}\right)\,\cos\left(\theta _{7}\right)\,\sin\left(\theta _{5}\right)\right) & -\sin\left(\theta _{3}\right)\,\left(\cos\left(\theta _{5}\right)\,\cos\left(\theta _{7}\right)-\cos\left(\theta _{6}\right)\,\sin\left(\theta _{5}\right)\,\sin\left(\theta _{7}\right)\right)-\cos\left(\theta _{3}\right)\,\sin\left(\theta _{4}\right)\,\left(\cos\left(\theta _{7}\right)\,\sin\left(\theta _{5}\right)+\cos\left(\theta _{5}\right)\,\cos\left(\theta _{6}\right)\,\sin\left(\theta _{7}\right)\right)-\cos\left(\theta _{3}\right)\,\cos\left(\theta _{4}\right)\,\sin\left(\theta _{6}\right)\,\sin\left(\theta _{7}\right) & \sin\left(\theta _{3}\right)\,\sin\left(\theta _{5}\right)\,\sin\left(\theta _{6}\right)+\cos\left(\theta _{3}\right)\,\cos\left(\theta _{4}\right)\,\cos\left(\theta _{6}\right)-\cos\left(\theta _{3}\right)\,\cos\left(\theta _{5}\right)\,\sin\left(\theta _{4}\right)\,\sin\left(\theta _{6}\right) & 200\,\cos\left(\theta _{3}\right)+\cos\left(\theta _{3}\right)\,\cos\left(\theta _{4}\right)\,\left(\cos\left(\theta _{6}\right)\,\left(\mathrm{Load}+129\right)+381\right)+\sin\left(\theta _{3}\right)\,\sin\left(\theta _{5}\right)\,\sin\left(\theta _{6}\right)\,\left(\mathrm{Load}+129\right)-\cos\left(\theta _{3}\right)\,\cos\left(\theta _{5}\right)\,\sin\left(\theta _{4}\right)\,\sin\left(\theta _{6}\right)\,\left(\mathrm{Load}+129\right)+90\\ \cos\left(\theta _{3}\right)\,\left(\cos\left(\theta _{5}\right)\,\sin\left(\theta _{7}\right)+\cos\left(\theta _{6}\right)\,\cos\left(\theta _{7}\right)\,\sin\left(\theta _{5}\right)\right)-\sin\left(\theta _{3}\right)\,\sin\left(\theta _{4}\right)\,\left(\sin\left(\theta _{5}\right)\,\sin\left(\theta _{7}\right)-\cos\left(\theta _{5}\right)\,\cos\left(\theta _{6}\right)\,\cos\left(\theta _{7}\right)\right)+\cos\left(\theta _{4}\right)\,\cos\left(\theta _{7}\right)\,\sin\left(\theta _{3}\right)\,\sin\left(\theta _{6}\right) & \cos\left(\theta _{3}\right)\,\left(\cos\left(\theta _{5}\right)\,\cos\left(\theta _{7}\right)-\cos\left(\theta _{6}\right)\,\sin\left(\theta _{5}\right)\,\sin\left(\theta _{7}\right)\right)-\sin\left(\theta _{3}\right)\,\sin\left(\theta _{4}\right)\,\left(\cos\left(\theta _{7}\right)\,\sin\left(\theta _{5}\right)+\cos\left(\theta _{5}\right)\,\cos\left(\theta _{6}\right)\,\sin\left(\theta _{7}\right)\right)-\cos\left(\theta _{4}\right)\,\sin\left(\theta _{3}\right)\,\sin\left(\theta _{6}\right)\,\sin\left(\theta _{7}\right) & \cos\left(\theta _{4}\right)\,\cos\left(\theta _{6}\right)\,\sin\left(\theta _{3}\right)-\cos\left(\theta _{3}\right)\,\sin\left(\theta _{5}\right)\,\sin\left(\theta _{6}\right)-\cos\left(\theta _{5}\right)\,\sin\left(\theta _{3}\right)\,\sin\left(\theta _{4}\right)\,\sin\left(\theta _{6}\right) & 200\,\sin\left(\theta _{3}\right)+\cos\left(\theta _{4}\right)\,\sin\left(\theta _{3}\right)\,\left(\cos\left(\theta _{6}\right)\,\left(\mathrm{Load}+129\right)+381\right)-\cos\left(\theta _{3}\right)\,\sin\left(\theta _{5}\right)\,\sin\left(\theta _{6}\right)\,\left(\mathrm{Load}+129\right)-\cos\left(\theta _{5}\right)\,\sin\left(\theta _{3}\right)\,\sin\left(\theta _{4}\right)\,\sin\left(\theta _{6}\right)\,\left(\mathrm{Load}+129\right)\\ \cos\left(\theta _{4}\right)\,\left(\sin\left(\theta _{5}\right)\,\sin\left(\theta _{7}\right)-\cos\left(\theta _{5}\right)\,\cos\left(\theta _{6}\right)\,\cos\left(\theta _{7}\right)\right)+\cos\left(\theta _{7}\right)\,\sin\left(\theta _{4}\right)\,\sin\left(\theta _{6}\right) & \cos\left(\theta _{4}\right)\,\left(\cos\left(\theta _{7}\right)\,\sin\left(\theta _{5}\right)+\cos\left(\theta _{5}\right)\,\cos\left(\theta _{6}\right)\,\sin\left(\theta _{7}\right)\right)-\sin\left(\theta _{4}\right)\,\sin\left(\theta _{6}\right)\,\sin\left(\theta _{7}\right) & \cos\left(\theta _{6}\right)\,\sin\left(\theta _{4}\right)+\cos\left(\theta _{4}\right)\,\cos\left(\theta _{5}\right)\,\sin\left(\theta _{6}\right) & \sin\left(\theta _{4}\right)\,\left(\cos\left(\theta _{6}\right)\,\left(\mathrm{Load}+129\right)+381\right)+\cos\left(\theta _{4}\right)\,\cos\left(\theta _{5}\right)\,\sin\left(\theta _{6}\right)\,\left(\mathrm{Load}+129\right)-\frac{591}{2}\\ 0 & 0 & 0 & 1 \end{array}\right)
$$

## T0i(6)

$$
\left(\begin{array}{cccc} \cos\left(\theta _{4}\right)\,\cos\left(\theta _{5}\right) & -\cos\left(\theta _{4}\right)\,\sin\left(\theta _{5}\right) & -\sin\left(\theta _{4}\right) & \frac{591}{2}-381\,\sin\left(\theta _{4}\right)-d_{2}\\ \sin\left(\theta _{3}\right)\,\sin\left(\theta _{5}\right)-\cos\left(\theta _{3}\right)\,\cos\left(\theta _{5}\right)\,\sin\left(\theta _{4}\right) & \cos\left(\theta _{5}\right)\,\sin\left(\theta _{3}\right)+\cos\left(\theta _{3}\right)\,\sin\left(\theta _{4}\right)\,\sin\left(\theta _{5}\right) & -\cos\left(\theta _{3}\right)\,\cos\left(\theta _{4}\right) & -d_{1}-200\,\cos\left(\theta _{3}\right)-381\,\cos\left(\theta _{3}\right)\,\cos\left(\theta _{4}\right)-90\\ \cos\left(\theta _{3}\right)\,\sin\left(\theta _{5}\right)+\cos\left(\theta _{5}\right)\,\sin\left(\theta _{3}\right)\,\sin\left(\theta _{4}\right) & \cos\left(\theta _{3}\right)\,\cos\left(\theta _{5}\right)-\sin\left(\theta _{3}\right)\,\sin\left(\theta _{4}\right)\,\sin\left(\theta _{5}\right) & \cos\left(\theta _{4}\right)\,\sin\left(\theta _{3}\right) & 200\,\sin\left(\theta _{3}\right)+381\,\cos\left(\theta _{4}\right)\,\sin\left(\theta _{3}\right)\\ 0 & 0 & 0 & 1 \end{array}\right)
$$

