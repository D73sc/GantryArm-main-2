# Homogeneous Transformation Matrices from Base to Joint

## T_10

$$
\left(\begin{array}{cccc} 0 & -1 & 0 & a_{0}\\ 0 & 0 & -1 & -d_{0}\\ 1 & 0 & 0 & 0\\ 0 & 0 & 0 & 1 \end{array}\right)
$$

## T_01

$$
\left(\begin{array}{cccc} 0 & -1 & 0 & a_{0}\\ 0 & 0 & -1 & -d_{0}\\ 1 & 0 & 0 & 0\\ 0 & 0 & 0 & 1 \end{array}\right)
$$

## T_21

$$
\left(\begin{array}{cccc} 0 & 1 & 0 & a_{1}\\ 0 & 0 & -1 & -d_{1}\\ -1 & 0 & 0 & 0\\ 0 & 0 & 0 & 1 \end{array}\right)
$$

## T_02

$$
\left(\begin{array}{cccc} 0 & 0 & 1 & a_{0}+d_{1}\\ 1 & 0 & 0 & -d_{0}\\ 0 & 1 & 0 & a_{1}\\ 0 & 0 & 0 & 1 \end{array}\right)
$$

## T_32

$$
\left(\begin{array}{cccc} 0 & -1 & 0 & a_{2}\\ 0 & 0 & -1 & -d_{2}\\ 1 & 0 & 0 & 0\\ 0 & 0 & 0 & 1 \end{array}\right)
$$

## T_03

$$
\left(\begin{array}{cccc} 1 & 0 & 0 & a_{0}+d_{1}\\ 0 & -1 & 0 & a_{2}-d_{0}\\ 0 & 0 & -1 & a_{1}-d_{2}\\ 0 & 0 & 0 & 1 \end{array}\right)
$$

## T_43

$$
\left(\begin{array}{cccc} \cos\left(\theta _{3}\right) & -\sin\left(\theta _{3}\right) & 0 & a_{3}\\ \sin\left(\theta _{3}\right) & \cos\left(\theta _{3}\right) & 0 & 0\\ 0 & 0 & 1 & -\frac{591}{2}\\ 0 & 0 & 0 & 1 \end{array}\right)
$$

## T_04

$$
\left(\begin{array}{cccc} \cos\left(\theta _{3}\right) & -\sin\left(\theta _{3}\right) & 0 & a_{0}+a_{3}+d_{1}\\ -\sin\left(\theta _{3}\right) & -\cos\left(\theta _{3}\right) & 0 & a_{2}-d_{0}\\ 0 & 0 & -1 & a_{1}-d_{2}+\frac{591}{2}\\ 0 & 0 & 0 & 1 \end{array}\right)
$$

## T_54

$$
\left(\begin{array}{cccc} -\sin\left(\theta _{4}\right) & -\cos\left(\theta _{4}\right) & 0 & a_{4}\\ 0 & 0 & -1 & 0\\ \cos\left(\theta _{4}\right) & -\sin\left(\theta _{4}\right) & 0 & 0\\ 0 & 0 & 0 & 1 \end{array}\right)
$$

## T_05

$$
\left(\begin{array}{cccc} -\cos\left(\theta _{3}\right)\,\sin\left(\theta _{4}\right) & -\cos\left(\theta _{3}\right)\,\cos\left(\theta _{4}\right) & \sin\left(\theta _{3}\right) & a_{0}+a_{3}+d_{1}+a_{4}\,\cos\left(\theta _{3}\right)\\ \sin\left(\theta _{3}\right)\,\sin\left(\theta _{4}\right) & \cos\left(\theta _{4}\right)\,\sin\left(\theta _{3}\right) & \cos\left(\theta _{3}\right) & a_{2}-d_{0}-a_{4}\,\sin\left(\theta _{3}\right)\\ -\cos\left(\theta _{4}\right) & \sin\left(\theta _{4}\right) & 0 & a_{1}-d_{2}+\frac{591}{2}\\ 0 & 0 & 0 & 1 \end{array}\right)
$$

## T_65

$$
\left(\begin{array}{cccc} -\cos\left(\theta _{5}\right) & \sin\left(\theta _{5}\right) & 0 & a_{5}\\ 0 & 0 & -1 & -381\\ -\sin\left(\theta _{5}\right) & -\cos\left(\theta _{5}\right) & 0 & 0\\ 0 & 0 & 0 & 1 \end{array}\right)
$$

## T_06

$$
\left(\begin{array}{cccc} \cos\left(\theta _{3}\right)\,\cos\left(\theta _{5}\right)\,\sin\left(\theta _{4}\right)-\sin\left(\theta _{3}\right)\,\sin\left(\theta _{5}\right) & -\cos\left(\theta _{5}\right)\,\sin\left(\theta _{3}\right)-\cos\left(\theta _{3}\right)\,\sin\left(\theta _{4}\right)\,\sin\left(\theta _{5}\right) & \cos\left(\theta _{3}\right)\,\cos\left(\theta _{4}\right) & a_{0}+a_{3}+d_{1}+381\,\cos\left(\theta _{3}\right)\,\cos\left(\theta _{4}\right)+a_{4}\,\cos\left(\theta _{3}\right)-a_{5}\,\cos\left(\theta _{3}\right)\,\sin\left(\theta _{4}\right)\\ -\cos\left(\theta _{3}\right)\,\sin\left(\theta _{5}\right)-\cos\left(\theta _{5}\right)\,\sin\left(\theta _{3}\right)\,\sin\left(\theta _{4}\right) & \sin\left(\theta _{3}\right)\,\sin\left(\theta _{4}\right)\,\sin\left(\theta _{5}\right)-\cos\left(\theta _{3}\right)\,\cos\left(\theta _{5}\right) & -\cos\left(\theta _{4}\right)\,\sin\left(\theta _{3}\right) & a_{2}-d_{0}-381\,\cos\left(\theta _{4}\right)\,\sin\left(\theta _{3}\right)-a_{4}\,\sin\left(\theta _{3}\right)+a_{5}\,\sin\left(\theta _{3}\right)\,\sin\left(\theta _{4}\right)\\ \cos\left(\theta _{4}\right)\,\cos\left(\theta _{5}\right) & -\cos\left(\theta _{4}\right)\,\sin\left(\theta _{5}\right) & -\sin\left(\theta _{4}\right) & a_{1}-d_{2}-381\,\sin\left(\theta _{4}\right)-a_{5}\,\cos\left(\theta _{4}\right)+\frac{591}{2}\\ 0 & 0 & 0 & 1 \end{array}\right)
$$

## T_76

$$
\left(\begin{array}{cccc} -\cos\left(\theta _{6}\right) & \sin\left(\theta _{6}\right) & 0 & a_{6}\\ 0 & 0 & -1 & 0\\ -\sin\left(\theta _{6}\right) & -\cos\left(\theta _{6}\right) & 0 & 0\\ 0 & 0 & 0 & 1 \end{array}\right)
$$

## T_07

$$
\left(\begin{array}{cccc} \cos\left(\theta _{6}\right)\,\left(\sin\left(\theta _{3}\right)\,\sin\left(\theta _{5}\right)-\cos\left(\theta _{3}\right)\,\cos\left(\theta _{5}\right)\,\sin\left(\theta _{4}\right)\right)-\cos\left(\theta _{3}\right)\,\cos\left(\theta _{4}\right)\,\sin\left(\theta _{6}\right) & -\sin\left(\theta _{6}\right)\,\left(\sin\left(\theta _{3}\right)\,\sin\left(\theta _{5}\right)-\cos\left(\theta _{3}\right)\,\cos\left(\theta _{5}\right)\,\sin\left(\theta _{4}\right)\right)-\cos\left(\theta _{3}\right)\,\cos\left(\theta _{4}\right)\,\cos\left(\theta _{6}\right) & \cos\left(\theta _{5}\right)\,\sin\left(\theta _{3}\right)+\cos\left(\theta _{3}\right)\,\sin\left(\theta _{4}\right)\,\sin\left(\theta _{5}\right) & a_{0}+a_{3}+d_{1}+381\,\cos\left(\theta _{3}\right)\,\cos\left(\theta _{4}\right)-a_{6}\,\left(\sin\left(\theta _{3}\right)\,\sin\left(\theta _{5}\right)-\cos\left(\theta _{3}\right)\,\cos\left(\theta _{5}\right)\,\sin\left(\theta _{4}\right)\right)+a_{4}\,\cos\left(\theta _{3}\right)-a_{5}\,\cos\left(\theta _{3}\right)\,\sin\left(\theta _{4}\right)\\ \cos\left(\theta _{6}\right)\,\left(\cos\left(\theta _{3}\right)\,\sin\left(\theta _{5}\right)+\cos\left(\theta _{5}\right)\,\sin\left(\theta _{3}\right)\,\sin\left(\theta _{4}\right)\right)+\cos\left(\theta _{4}\right)\,\sin\left(\theta _{3}\right)\,\sin\left(\theta _{6}\right) & \cos\left(\theta _{4}\right)\,\cos\left(\theta _{6}\right)\,\sin\left(\theta _{3}\right)-\sin\left(\theta _{6}\right)\,\left(\cos\left(\theta _{3}\right)\,\sin\left(\theta _{5}\right)+\cos\left(\theta _{5}\right)\,\sin\left(\theta _{3}\right)\,\sin\left(\theta _{4}\right)\right) & \cos\left(\theta _{3}\right)\,\cos\left(\theta _{5}\right)-\sin\left(\theta _{3}\right)\,\sin\left(\theta _{4}\right)\,\sin\left(\theta _{5}\right) & a_{2}-d_{0}-381\,\cos\left(\theta _{4}\right)\,\sin\left(\theta _{3}\right)-a_{6}\,\left(\cos\left(\theta _{3}\right)\,\sin\left(\theta _{5}\right)+\cos\left(\theta _{5}\right)\,\sin\left(\theta _{3}\right)\,\sin\left(\theta _{4}\right)\right)-a_{4}\,\sin\left(\theta _{3}\right)+a_{5}\,\sin\left(\theta _{3}\right)\,\sin\left(\theta _{4}\right)\\ \sin\left(\theta _{4}\right)\,\sin\left(\theta _{6}\right)-\cos\left(\theta _{4}\right)\,\cos\left(\theta _{5}\right)\,\cos\left(\theta _{6}\right) & \cos\left(\theta _{6}\right)\,\sin\left(\theta _{4}\right)+\cos\left(\theta _{4}\right)\,\cos\left(\theta _{5}\right)\,\sin\left(\theta _{6}\right) & \cos\left(\theta _{4}\right)\,\sin\left(\theta _{5}\right) & a_{1}-d_{2}-381\,\sin\left(\theta _{4}\right)-a_{5}\,\cos\left(\theta _{4}\right)+a_{6}\,\cos\left(\theta _{4}\right)\,\cos\left(\theta _{5}\right)+\frac{591}{2}\\ 0 & 0 & 0 & 1 \end{array}\right)
$$

## T_87

$$
\left(\begin{array}{cccc} -\cos\left(\theta _{7}\right) & \sin\left(\theta _{7}\right) & 0 & a_{7}\\ 0 & 0 & -1 & -\mathrm{Load}-129\\ -\sin\left(\theta _{7}\right) & -\cos\left(\theta _{7}\right) & 0 & 0\\ 0 & 0 & 0 & 1 \end{array}\right)
$$

## T_08

$$
\left(\begin{array}{cccc} -\cos\left(\theta _{7}\right)\,\left(\cos\left(\theta _{6}\right)\,\left(\sin\left(\theta _{3}\right)\,\sin\left(\theta _{5}\right)-\cos\left(\theta _{3}\right)\,\cos\left(\theta _{5}\right)\,\sin\left(\theta _{4}\right)\right)-\cos\left(\theta _{3}\right)\,\cos\left(\theta _{4}\right)\,\sin\left(\theta _{6}\right)\right)-\sin\left(\theta _{7}\right)\,\left(\cos\left(\theta _{5}\right)\,\sin\left(\theta _{3}\right)+\cos\left(\theta _{3}\right)\,\sin\left(\theta _{4}\right)\,\sin\left(\theta _{5}\right)\right) & \sin\left(\theta _{7}\right)\,\left(\cos\left(\theta _{6}\right)\,\left(\sin\left(\theta _{3}\right)\,\sin\left(\theta _{5}\right)-\cos\left(\theta _{3}\right)\,\cos\left(\theta _{5}\right)\,\sin\left(\theta _{4}\right)\right)-\cos\left(\theta _{3}\right)\,\cos\left(\theta _{4}\right)\,\sin\left(\theta _{6}\right)\right)-\cos\left(\theta _{7}\right)\,\left(\cos\left(\theta _{5}\right)\,\sin\left(\theta _{3}\right)+\cos\left(\theta _{3}\right)\,\sin\left(\theta _{4}\right)\,\sin\left(\theta _{5}\right)\right) & \sin\left(\theta _{6}\right)\,\left(\sin\left(\theta _{3}\right)\,\sin\left(\theta _{5}\right)-\cos\left(\theta _{3}\right)\,\cos\left(\theta _{5}\right)\,\sin\left(\theta _{4}\right)\right)+\cos\left(\theta _{3}\right)\,\cos\left(\theta _{4}\right)\,\cos\left(\theta _{6}\right) & a_{0}+a_{3}+d_{1}+381\,\cos\left(\theta _{3}\right)\,\cos\left(\theta _{4}\right)+\left(\sin\left(\theta _{6}\right)\,\left(\sin\left(\theta _{3}\right)\,\sin\left(\theta _{5}\right)-\cos\left(\theta _{3}\right)\,\cos\left(\theta _{5}\right)\,\sin\left(\theta _{4}\right)\right)+\cos\left(\theta _{3}\right)\,\cos\left(\theta _{4}\right)\,\cos\left(\theta _{6}\right)\right)\,\left(\mathrm{Load}+129\right)-a_{6}\,\left(\sin\left(\theta _{3}\right)\,\sin\left(\theta _{5}\right)-\cos\left(\theta _{3}\right)\,\cos\left(\theta _{5}\right)\,\sin\left(\theta _{4}\right)\right)+a_{4}\,\cos\left(\theta _{3}\right)+a_{7}\,\left(\cos\left(\theta _{6}\right)\,\left(\sin\left(\theta _{3}\right)\,\sin\left(\theta _{5}\right)-\cos\left(\theta _{3}\right)\,\cos\left(\theta _{5}\right)\,\sin\left(\theta _{4}\right)\right)-\cos\left(\theta _{3}\right)\,\cos\left(\theta _{4}\right)\,\sin\left(\theta _{6}\right)\right)-a_{5}\,\cos\left(\theta _{3}\right)\,\sin\left(\theta _{4}\right)\\ -\cos\left(\theta _{7}\right)\,\left(\cos\left(\theta _{6}\right)\,\left(\cos\left(\theta _{3}\right)\,\sin\left(\theta _{5}\right)+\cos\left(\theta _{5}\right)\,\sin\left(\theta _{3}\right)\,\sin\left(\theta _{4}\right)\right)+\cos\left(\theta _{4}\right)\,\sin\left(\theta _{3}\right)\,\sin\left(\theta _{6}\right)\right)-\sin\left(\theta _{7}\right)\,\left(\cos\left(\theta _{3}\right)\,\cos\left(\theta _{5}\right)-\sin\left(\theta _{3}\right)\,\sin\left(\theta _{4}\right)\,\sin\left(\theta _{5}\right)\right) & \sin\left(\theta _{7}\right)\,\left(\cos\left(\theta _{6}\right)\,\left(\cos\left(\theta _{3}\right)\,\sin\left(\theta _{5}\right)+\cos\left(\theta _{5}\right)\,\sin\left(\theta _{3}\right)\,\sin\left(\theta _{4}\right)\right)+\cos\left(\theta _{4}\right)\,\sin\left(\theta _{3}\right)\,\sin\left(\theta _{6}\right)\right)-\cos\left(\theta _{7}\right)\,\left(\cos\left(\theta _{3}\right)\,\cos\left(\theta _{5}\right)-\sin\left(\theta _{3}\right)\,\sin\left(\theta _{4}\right)\,\sin\left(\theta _{5}\right)\right) & \sin\left(\theta _{6}\right)\,\left(\cos\left(\theta _{3}\right)\,\sin\left(\theta _{5}\right)+\cos\left(\theta _{5}\right)\,\sin\left(\theta _{3}\right)\,\sin\left(\theta _{4}\right)\right)-\cos\left(\theta _{4}\right)\,\cos\left(\theta _{6}\right)\,\sin\left(\theta _{3}\right) & a_{2}-d_{0}-381\,\cos\left(\theta _{4}\right)\,\sin\left(\theta _{3}\right)+\left(\sin\left(\theta _{6}\right)\,\left(\cos\left(\theta _{3}\right)\,\sin\left(\theta _{5}\right)+\cos\left(\theta _{5}\right)\,\sin\left(\theta _{3}\right)\,\sin\left(\theta _{4}\right)\right)-\cos\left(\theta _{4}\right)\,\cos\left(\theta _{6}\right)\,\sin\left(\theta _{3}\right)\right)\,\left(\mathrm{Load}+129\right)-a_{6}\,\left(\cos\left(\theta _{3}\right)\,\sin\left(\theta _{5}\right)+\cos\left(\theta _{5}\right)\,\sin\left(\theta _{3}\right)\,\sin\left(\theta _{4}\right)\right)-a_{4}\,\sin\left(\theta _{3}\right)+a_{7}\,\left(\cos\left(\theta _{6}\right)\,\left(\cos\left(\theta _{3}\right)\,\sin\left(\theta _{5}\right)+\cos\left(\theta _{5}\right)\,\sin\left(\theta _{3}\right)\,\sin\left(\theta _{4}\right)\right)+\cos\left(\theta _{4}\right)\,\sin\left(\theta _{3}\right)\,\sin\left(\theta _{6}\right)\right)+a_{5}\,\sin\left(\theta _{3}\right)\,\sin\left(\theta _{4}\right)\\ -\cos\left(\theta _{7}\right)\,\left(\sin\left(\theta _{4}\right)\,\sin\left(\theta _{6}\right)-\cos\left(\theta _{4}\right)\,\cos\left(\theta _{5}\right)\,\cos\left(\theta _{6}\right)\right)-\cos\left(\theta _{4}\right)\,\sin\left(\theta _{5}\right)\,\sin\left(\theta _{7}\right) & \sin\left(\theta _{7}\right)\,\left(\sin\left(\theta _{4}\right)\,\sin\left(\theta _{6}\right)-\cos\left(\theta _{4}\right)\,\cos\left(\theta _{5}\right)\,\cos\left(\theta _{6}\right)\right)-\cos\left(\theta _{4}\right)\,\cos\left(\theta _{7}\right)\,\sin\left(\theta _{5}\right) & -\cos\left(\theta _{6}\right)\,\sin\left(\theta _{4}\right)-\cos\left(\theta _{4}\right)\,\cos\left(\theta _{5}\right)\,\sin\left(\theta _{6}\right) & a_{1}-d_{2}-381\,\sin\left(\theta _{4}\right)+a_{7}\,\left(\sin\left(\theta _{4}\right)\,\sin\left(\theta _{6}\right)-\cos\left(\theta _{4}\right)\,\cos\left(\theta _{5}\right)\,\cos\left(\theta _{6}\right)\right)-\left(\cos\left(\theta _{6}\right)\,\sin\left(\theta _{4}\right)+\cos\left(\theta _{4}\right)\,\cos\left(\theta _{5}\right)\,\sin\left(\theta _{6}\right)\right)\,\left(\mathrm{Load}+129\right)-a_{5}\,\cos\left(\theta _{4}\right)+a_{6}\,\cos\left(\theta _{4}\right)\,\cos\left(\theta _{5}\right)+\frac{591}{2}\\ 0 & 0 & 0 & 1 \end{array}\right)
$$

## T_0_3_inv

$$
\left(\begin{array}{cccc} 1 & 0 & 0 & -a_{0}-d_{1}\\ 0 & -1 & 0 & a_{2}-d_{0}\\ 0 & 0 & -1 & a_{1}-d_{2}\\ 0 & 0 & 0 & 1 \end{array}\right)
$$

## T_3_5

$$
\left(\begin{array}{cccc} -\cos\left(\theta _{3}\right)\,\sin\left(\theta _{4}\right) & -\cos\left(\theta _{3}\right)\,\cos\left(\theta _{4}\right) & \sin\left(\theta _{3}\right) & a_{3}+a_{4}\,\cos\left(\theta _{3}\right)\\ -\sin\left(\theta _{3}\right)\,\sin\left(\theta _{4}\right) & -\cos\left(\theta _{4}\right)\,\sin\left(\theta _{3}\right) & -\cos\left(\theta _{3}\right) & a_{4}\,\sin\left(\theta _{3}\right)\\ \cos\left(\theta _{4}\right) & -\sin\left(\theta _{4}\right) & 0 & -\frac{591}{2}\\ 0 & 0 & 0 & 1 \end{array}\right)
$$

## T_5_8

$$
\left(\begin{array}{cccc} \sin\left(\theta _{5}\right)\,\sin\left(\theta _{7}\right)-\cos\left(\theta _{5}\right)\,\cos\left(\theta _{6}\right)\,\cos\left(\theta _{7}\right) & \cos\left(\theta _{7}\right)\,\sin\left(\theta _{5}\right)+\cos\left(\theta _{5}\right)\,\cos\left(\theta _{6}\right)\,\sin\left(\theta _{7}\right) & \cos\left(\theta _{5}\right)\,\sin\left(\theta _{6}\right) & a_{5}-a_{6}\,\cos\left(\theta _{5}\right)+a_{7}\,\cos\left(\theta _{5}\right)\,\cos\left(\theta _{6}\right)+\cos\left(\theta _{5}\right)\,\sin\left(\theta _{6}\right)\,\left(\mathrm{Load}+129\right)\\ -\cos\left(\theta _{7}\right)\,\sin\left(\theta _{6}\right) & \sin\left(\theta _{6}\right)\,\sin\left(\theta _{7}\right) & -\cos\left(\theta _{6}\right) & a_{7}\,\sin\left(\theta _{6}\right)-\cos\left(\theta _{6}\right)\,\left(\mathrm{Load}+129\right)-381\\ -\cos\left(\theta _{5}\right)\,\sin\left(\theta _{7}\right)-\cos\left(\theta _{6}\right)\,\cos\left(\theta _{7}\right)\,\sin\left(\theta _{5}\right) & \cos\left(\theta _{6}\right)\,\sin\left(\theta _{5}\right)\,\sin\left(\theta _{7}\right)-\cos\left(\theta _{5}\right)\,\cos\left(\theta _{7}\right) & \sin\left(\theta _{5}\right)\,\sin\left(\theta _{6}\right) & a_{7}\,\cos\left(\theta _{6}\right)\,\sin\left(\theta _{5}\right)-a_{6}\,\sin\left(\theta _{5}\right)+\sin\left(\theta _{5}\right)\,\sin\left(\theta _{6}\right)\,\left(\mathrm{Load}+129\right)\\ 0 & 0 & 0 & 1 \end{array}\right)
$$

## T_3_8

$$
\left(\begin{array}{cccc} \cos\left(\theta _{3}\right)\,\cos\left(\theta _{4}\right)\,\cos\left(\theta _{7}\right)\,\sin\left(\theta _{6}\right)-\cos\left(\theta _{3}\right)\,\sin\left(\theta _{4}\right)\,\left(\sin\left(\theta _{5}\right)\,\sin\left(\theta _{7}\right)-\cos\left(\theta _{5}\right)\,\cos\left(\theta _{6}\right)\,\cos\left(\theta _{7}\right)\right)-\sin\left(\theta _{3}\right)\,\left(\cos\left(\theta _{5}\right)\,\sin\left(\theta _{7}\right)+\cos\left(\theta _{6}\right)\,\cos\left(\theta _{7}\right)\,\sin\left(\theta _{5}\right)\right) & -\sin\left(\theta _{3}\right)\,\left(\cos\left(\theta _{5}\right)\,\cos\left(\theta _{7}\right)-\cos\left(\theta _{6}\right)\,\sin\left(\theta _{5}\right)\,\sin\left(\theta _{7}\right)\right)-\cos\left(\theta _{3}\right)\,\sin\left(\theta _{4}\right)\,\left(\cos\left(\theta _{7}\right)\,\sin\left(\theta _{5}\right)+\cos\left(\theta _{5}\right)\,\cos\left(\theta _{6}\right)\,\sin\left(\theta _{7}\right)\right)-\cos\left(\theta _{3}\right)\,\cos\left(\theta _{4}\right)\,\sin\left(\theta _{6}\right)\,\sin\left(\theta _{7}\right) & \sin\left(\theta _{3}\right)\,\sin\left(\theta _{5}\right)\,\sin\left(\theta _{6}\right)+\cos\left(\theta _{3}\right)\,\cos\left(\theta _{4}\right)\,\cos\left(\theta _{6}\right)-\cos\left(\theta _{3}\right)\,\cos\left(\theta _{5}\right)\,\sin\left(\theta _{4}\right)\,\sin\left(\theta _{6}\right) & a_{3}+a_{4}\,\cos\left(\theta _{3}\right)+\sin\left(\theta _{3}\right)\,\left(a_{7}\,\cos\left(\theta _{6}\right)\,\sin\left(\theta _{5}\right)-a_{6}\,\sin\left(\theta _{5}\right)+\sin\left(\theta _{5}\right)\,\sin\left(\theta _{6}\right)\,\left(\mathrm{Load}+129\right)\right)-\cos\left(\theta _{3}\right)\,\sin\left(\theta _{4}\right)\,\left(a_{5}-a_{6}\,\cos\left(\theta _{5}\right)+a_{7}\,\cos\left(\theta _{5}\right)\,\cos\left(\theta _{6}\right)+\cos\left(\theta _{5}\right)\,\sin\left(\theta _{6}\right)\,\left(\mathrm{Load}+129\right)\right)+\cos\left(\theta _{3}\right)\,\cos\left(\theta _{4}\right)\,\left(\cos\left(\theta _{6}\right)\,\left(\mathrm{Load}+129\right)-a_{7}\,\sin\left(\theta _{6}\right)+381\right)\\ \cos\left(\theta _{3}\right)\,\left(\cos\left(\theta _{5}\right)\,\sin\left(\theta _{7}\right)+\cos\left(\theta _{6}\right)\,\cos\left(\theta _{7}\right)\,\sin\left(\theta _{5}\right)\right)-\sin\left(\theta _{3}\right)\,\sin\left(\theta _{4}\right)\,\left(\sin\left(\theta _{5}\right)\,\sin\left(\theta _{7}\right)-\cos\left(\theta _{5}\right)\,\cos\left(\theta _{6}\right)\,\cos\left(\theta _{7}\right)\right)+\cos\left(\theta _{4}\right)\,\cos\left(\theta _{7}\right)\,\sin\left(\theta _{3}\right)\,\sin\left(\theta _{6}\right) & \cos\left(\theta _{3}\right)\,\left(\cos\left(\theta _{5}\right)\,\cos\left(\theta _{7}\right)-\cos\left(\theta _{6}\right)\,\sin\left(\theta _{5}\right)\,\sin\left(\theta _{7}\right)\right)-\sin\left(\theta _{3}\right)\,\sin\left(\theta _{4}\right)\,\left(\cos\left(\theta _{7}\right)\,\sin\left(\theta _{5}\right)+\cos\left(\theta _{5}\right)\,\cos\left(\theta _{6}\right)\,\sin\left(\theta _{7}\right)\right)-\cos\left(\theta _{4}\right)\,\sin\left(\theta _{3}\right)\,\sin\left(\theta _{6}\right)\,\sin\left(\theta _{7}\right) & \cos\left(\theta _{4}\right)\,\cos\left(\theta _{6}\right)\,\sin\left(\theta _{3}\right)-\cos\left(\theta _{3}\right)\,\sin\left(\theta _{5}\right)\,\sin\left(\theta _{6}\right)-\cos\left(\theta _{5}\right)\,\sin\left(\theta _{3}\right)\,\sin\left(\theta _{4}\right)\,\sin\left(\theta _{6}\right) & a_{4}\,\sin\left(\theta _{3}\right)-\cos\left(\theta _{3}\right)\,\left(a_{7}\,\cos\left(\theta _{6}\right)\,\sin\left(\theta _{5}\right)-a_{6}\,\sin\left(\theta _{5}\right)+\sin\left(\theta _{5}\right)\,\sin\left(\theta _{6}\right)\,\left(\mathrm{Load}+129\right)\right)+\cos\left(\theta _{4}\right)\,\sin\left(\theta _{3}\right)\,\left(\cos\left(\theta _{6}\right)\,\left(\mathrm{Load}+129\right)-a_{7}\,\sin\left(\theta _{6}\right)+381\right)-\sin\left(\theta _{3}\right)\,\sin\left(\theta _{4}\right)\,\left(a_{5}-a_{6}\,\cos\left(\theta _{5}\right)+a_{7}\,\cos\left(\theta _{5}\right)\,\cos\left(\theta _{6}\right)+\cos\left(\theta _{5}\right)\,\sin\left(\theta _{6}\right)\,\left(\mathrm{Load}+129\right)\right)\\ \cos\left(\theta _{4}\right)\,\left(\sin\left(\theta _{5}\right)\,\sin\left(\theta _{7}\right)-\cos\left(\theta _{5}\right)\,\cos\left(\theta _{6}\right)\,\cos\left(\theta _{7}\right)\right)+\cos\left(\theta _{7}\right)\,\sin\left(\theta _{4}\right)\,\sin\left(\theta _{6}\right) & \cos\left(\theta _{4}\right)\,\left(\cos\left(\theta _{7}\right)\,\sin\left(\theta _{5}\right)+\cos\left(\theta _{5}\right)\,\cos\left(\theta _{6}\right)\,\sin\left(\theta _{7}\right)\right)-\sin\left(\theta _{4}\right)\,\sin\left(\theta _{6}\right)\,\sin\left(\theta _{7}\right) & \cos\left(\theta _{6}\right)\,\sin\left(\theta _{4}\right)+\cos\left(\theta _{4}\right)\,\cos\left(\theta _{5}\right)\,\sin\left(\theta _{6}\right) & \sin\left(\theta _{4}\right)\,\left(\cos\left(\theta _{6}\right)\,\left(\mathrm{Load}+129\right)-a_{7}\,\sin\left(\theta _{6}\right)+381\right)+\cos\left(\theta _{4}\right)\,\left(a_{5}-a_{6}\,\cos\left(\theta _{5}\right)+a_{7}\,\cos\left(\theta _{5}\right)\,\cos\left(\theta _{6}\right)+\cos\left(\theta _{5}\right)\,\sin\left(\theta _{6}\right)\,\left(\mathrm{Load}+129\right)\right)-\frac{591}{2}\\ 0 & 0 & 0 & 1 \end{array}\right)
$$

## T0i(6)

$$
\left(\begin{array}{cccc} \cos\left(\theta _{3}\right)\,\cos\left(\theta _{5}\right)\,\sin\left(\theta _{4}\right)-\sin\left(\theta _{3}\right)\,\sin\left(\theta _{5}\right) & -\cos\left(\theta _{5}\right)\,\sin\left(\theta _{3}\right)-\cos\left(\theta _{3}\right)\,\sin\left(\theta _{4}\right)\,\sin\left(\theta _{5}\right) & \cos\left(\theta _{3}\right)\,\cos\left(\theta _{4}\right) & a_{0}+a_{3}+d_{1}+381\,\cos\left(\theta _{3}\right)\,\cos\left(\theta _{4}\right)+a_{4}\,\cos\left(\theta _{3}\right)-a_{5}\,\cos\left(\theta _{3}\right)\,\sin\left(\theta _{4}\right)\\ -\cos\left(\theta _{3}\right)\,\sin\left(\theta _{5}\right)-\cos\left(\theta _{5}\right)\,\sin\left(\theta _{3}\right)\,\sin\left(\theta _{4}\right) & \sin\left(\theta _{3}\right)\,\sin\left(\theta _{4}\right)\,\sin\left(\theta _{5}\right)-\cos\left(\theta _{3}\right)\,\cos\left(\theta _{5}\right) & -\cos\left(\theta _{4}\right)\,\sin\left(\theta _{3}\right) & a_{2}-d_{0}-381\,\cos\left(\theta _{4}\right)\,\sin\left(\theta _{3}\right)-a_{4}\,\sin\left(\theta _{3}\right)+a_{5}\,\sin\left(\theta _{3}\right)\,\sin\left(\theta _{4}\right)\\ \cos\left(\theta _{4}\right)\,\cos\left(\theta _{5}\right) & -\cos\left(\theta _{4}\right)\,\sin\left(\theta _{5}\right) & -\sin\left(\theta _{4}\right) & a_{1}-d_{2}-381\,\sin\left(\theta _{4}\right)-a_{5}\,\cos\left(\theta _{4}\right)+\frac{591}{2}\\ 0 & 0 & 0 & 1 \end{array}\right)
$$

