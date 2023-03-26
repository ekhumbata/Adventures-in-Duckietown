# Exercise 5 Report

## TL;DR

In this lab 
<br>

## Part 1
---
### Backpropagation Step by Step

<br>

Using the helpful guide of [Backpropagation Step by Step](https://hmkcode.com/ai/backpropagation-step-by-step/), we can do another backwards pass through the given neural network to reduce the prediction error. 

We start off with a matrix looking like the following after first pass:

$$\begin{equation}
\begin{bmatrix}
\textcolor{yellow}2 & \textcolor{yellow}3
\end{bmatrix}
\cdot
\begin{bmatrix}
\textcolor{lightblue}{0.12} & \textcolor{orange}{0.13} \\
\textcolor{lightblue}{0.23} & \textcolor{orange}{0.10}
\end{bmatrix}
= 
\begin{bmatrix}
\textcolor{teal}{0.92} & \textcolor{teal}{0.56}
\end{bmatrix}
\cdot
\begin{bmatrix}
\textcolor{grey}{0.17} \\
\textcolor{grey}{0.17} 
\end{bmatrix}
= 
\begin{bmatrix}
\textcolor{green}{0.26}
\end{bmatrix}
\end{equation}
$$

Where:
- $\textcolor{yellow}{i_1} = 2$
- $\textcolor{yellow}{i_2} = 3$
- $\textcolor{lightblue}{w_1} = 0.12$
- $\textcolor{lightblue}{w_2} = 0.23$
- $\textcolor{orange}{w_3} = 0.13$
- $\textcolor{orange}{w_4} = 0.10$
- $\textcolor{teal}{h_1} = 0.92$
- $\textcolor{teal}{h_2} = 0.56$
- $\textcolor{grey}{w_5} = 0.17$
- $\textcolor{grey}{w_6} = 0.17$
- $\textcolor{green}{prediction} = 0.26$

Now, we'll perform a another backward pass through the network so we can find new weights.

For this backwards pass we'll set our learning rate, $\textcolor{pink}{a} = 0.05$

We'll also find a new $\Delta = \textcolor{green}{prediction} - \textcolor{purple}{actual} = \textcolor{green}{0.26} -\textcolor{purple}{1} = \text{-}0.74$

Using the formulas provided in the documentation we can find:

$$\begin{equation}
\begin{bmatrix}
\textcolor{grey}{\bold{w_5}}\\
\textcolor{grey}{\bold{w_6}}
\end{bmatrix}
=
\begin{bmatrix}
\textcolor{grey}{0.17}\\
\textcolor{grey}{0.17}
\end{bmatrix}
-
\textcolor{pink}{0.05}
\text{(-0.74)}
\begin{bmatrix}
\textcolor{teal}{0.92}\\
\textcolor{teal}{0.56}
\end{bmatrix}
=
\begin{bmatrix}
\textcolor{grey}{0.17}\\
\textcolor{grey}{0.17}
\end{bmatrix}
-
\begin{bmatrix}
\text{-0.034}\\
\text{-0.021}
\end{bmatrix}
=
\begin{bmatrix}
\textcolor{grey}{0.20}\\
\textcolor{grey}{0.19}
\end{bmatrix}
\end{equation}
$$

<br>

$$\begin{equation}
\begin{bmatrix}
\textcolor{lightblue}{\bold{w_1}} & \textcolor{orange}{\bold{w_3}}\\
\textcolor{lightblue}{\bold{w_2}} & \textcolor{orange}{\bold{w_4}}
\end{bmatrix}
=
\begin{bmatrix}
\textcolor{lightblue}{0.12} & \textcolor{orange}{0.13}\\
\textcolor{lightblue}{0.23} & \textcolor{orange}{0.10}
\end{bmatrix}
-
\textcolor{pink}{0.05}
\text{(-0.74)}
\begin{bmatrix}
\textcolor{yellow}{2}\\
\textcolor{yellow}{3}
\end{bmatrix}
\begin{bmatrix}
\textcolor{grey}{0.17} & \textcolor{grey}{0.17}
\end{bmatrix}
=
\begin{bmatrix}
\textcolor{lightblue}{0.12} & \textcolor{orange}{0.13}\\
\textcolor{lightblue}{0.23} & \textcolor{orange}{0.10}
\end{bmatrix}
-
\begin{bmatrix}
-0.013 & -0.013\\
-0.019 & -0.019
\end{bmatrix}
=
\begin{bmatrix}
\textcolor{lightblue}{0.13} & \textcolor{orange}{0.14}\\
\textcolor{lightblue}{0.25} & \textcolor{orange}{0.12}
\end{bmatrix}
\end{equation}
$$

Now with the new weights calculated, we can do a forward pass to make a new prediction.
$$\begin{equation}
\begin{bmatrix}
\textcolor{yellow}2 & \textcolor{yellow}3
\end{bmatrix}
\cdot
\begin{bmatrix}
\textcolor{lightblue}{0.13} & \textcolor{orange}{0.14} \\
\textcolor{lightblue}{0.25} & \textcolor{orange}{0.12}
\end{bmatrix}
= 
\begin{bmatrix}
\textcolor{teal}{1.01} & \textcolor{teal}{0.64}
\end{bmatrix}
\cdot
\begin{bmatrix}
\textcolor{grey}{0.20} \\
\textcolor{grey}{0.19} 
\end{bmatrix}
= 
\begin{bmatrix}
\textcolor{green}{0.32}
\end{bmatrix}
\end{equation}
$$

During our initial pass, we had an $\textcolor{red}{error} = 0.327$

For this pass we have we'll find a new error:
$$
\begin{equation}
\textcolor{red}{error} = \frac{1}{2} \cdot (\textcolor{green}{prediction} - \textcolor{purple}{actual})^2 = \textcolor{green}{0.32} -\textcolor{purple}{1} = 0.228
\end{equation}
$$

This results in a reduction of error of ~9.9% vs. the first pass.

<br>

## Part 2
---
### Multilayer Perceptron

<br>

**What data augmentation is used in training? Please delete the data augmentation and rerun the code to compare.**

Answer 1

<br>

**What is the batch size in the code? Please change the batch size to 16 and 1024 and explain the variation in results.**

Answer 2

<br>

**What activation function is used in the hidden layer? Please replace it with the linear activation function and see how the training output differs. Show your results before and after changing the activation function in your written report.**

Answer 3

<br>

**What is the optimization algorithm in the code? Explain the role of optimization algorithm in training process.**

Answer 4

<br>

**Add dropout in the training and explain how the dropout layer helps in training.**

Answer 5

<br>

## Part 3
---
### Implement It!

Below you can see the 

![]()
<figcaption align = "center"><b>Vid.1: Duckiebot detecting numbers</b></figcaption>

<br>

**What was your strategy for implementing number detection using ML?**

Answer 4

<br>

**How well did your implemented strategy work? Was it reliable? In what situations did it perform poorly?**

Answer 4


<br>


<br>

## References
---
[Lab Manual](https://eclass.srv.ualberta.ca/pluginfile.php/9335412/mod_resource/content/1/Exercise%205.pdf)

[dt-core Library](https://github.com/duckietown/dt-core)

[CMPUT 412 Cheat Sheet](https://docs.google.com/document/d/1bQfkR_tmwctFozEZlZkmojBZHkegscJPJVuw-IEXwI4/edit#)

[Backpropagation Step by Step](https://hmkcode.com/ai/backpropagation-step-by-step/)

