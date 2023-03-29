# Exercise 5 Report

## TL;DR

MNIST is good, but maybe not the best when things are moving and at angles. In this lab, we implemented machine learning on the duckiebot to detect numbers while (almost always) following the rules of the duckietown road.
<br>

## Part 1
---
### Backpropagation Step by Step

<br>

Using the helpful guide of [Backpropagation Step by Step](https://hmkcode.com/ai/backpropagation-step-by-step/), we can do another backwards pass through the given neural network to reduce the prediction error. 

We start off with a matrix looking like the following after first pass:

$$\begin{equation}
\begin{bmatrix}
\textcolor{brown}2 & \textcolor{brown}3
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
- $\textcolor{brown}{i_1} = 2$
- $\textcolor{brown}{i_2} = 3$
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
\textcolor{brown}{2}\\
\textcolor{brown}{3}
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
\textcolor{brown}2 & \textcolor{brown}3
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
\textcolor{red}{error} = \frac{1}{2} \cdot (\textcolor{green}{prediction} - \textcolor{purple}{actual})^2 = \frac{1}{2} \cdot (\textcolor{green}{0.32} - \textcolor{purple}{1})^2 = 0.228
\end{equation}
$$

This results in a reduction of error of ~9.9% vs. the first pass.

<br>

## Part 2
---
### Multilayer Perceptron

<br>

**What data augmentation is used in training? Please delete the data augmentation and rerun the code to compare.**

We perform two augmentations on the data, rotating the images by +/- 5 degrees, as well as taking only a portion of the image by random cropping. Removing these from the code had little effect on the test and validation accuracy since we applied transforms to all the data sets when training them the first time. After removing the transforms none of the data was augmented. This made was no real difference in test, however, since we did not alter our dataset at all, our model will have a harder time generalizing to unseen data.

<br>

**What is the batch size in the code? Please change the batch size to 16 and 1024 and explain the variation in results.**

The batch size, $b$, is the number of forward passes we make on the model, storing the loss and error each time, before we average the $b$ costs together to make our update on the weights in the model. This means that for a smaller $b$, we would get a less stochastic update, since we are making updates more specific to each of the seen data points. This however, this takes longer since we need to make more updates to get through the entire dataset. 

The inverse is true about a large batch size. We take a noisier step in the direction of the gradient, since we are averaging across a larger set of data points, but this allows us to make fewer updates to get through the dataset. The more accurate steps seen with a smaller batch size increases our convergence rate and thus will allow us to achieve a more accurate model with the same number of epochs.

Thus, when we reduce the batch size to 16 we see slower training times (25 secs/epoch), and a better test accuracy (97.79%), and for the larger 1024 batch size we get shorter training times  (10 secs/epoch), but a reduced model accuracy (97.47%). 

<br>

**What activation function is used in the hidden layer? Please replace it with the linear activation function and see how the training output differs. Show your results before and after changing the activation function in your written report.**

The ReLU activation function is used by default in this example. With that activation function, and a batch size of 1024 we achieved train accuracy of 97.38%. After changing the activation function to a linear activation, we saw a decrease in performance, with a 20.88% train accuracy. The reduction in accuracy we see is because ReLU is a nonlinear function, and since most of the underlying functions we are trying to represent (in this case the function that maps images to numbers) are non linear, it makes sense that having an activation function that is also nonlinear will perform better. 

<br>

**What is the optimization algorithm in the code? Explain the role of optimization algorithm in training process.**

The optimizer in this example is Adam. The role of this optimizer is to help us choose a variable step size that can adjust for where we are in the gradient. In this case, Adam incorporates a momentum into our step size, which helps us to accelerate towards minima. In other words, optimizers help us take better steps towards our goals, larger steps when we are far from a solution, and smaller steps when we are near minima.

<br>

**Add dropout in the training and explain how the dropout layer helps in training.**

Adding a dropout layer reduces the correlation between nodes within a layer, and thus improves performance of our model, since it means we will neglect to update all the weights in one layer on every pass. It also acts as a form of regularization, forcing our model to choose weights with smaller magnitudes which is also advantageous since it will reduce the likelihood of overfitting to our training data, and increase generalizability. Dropout layers also usually decrease our convergence rates meaning we will need more epochs in order to achieve a comparable model. Due to this, we used 20 epochs after implementing the dropout layers as opposed to 10. Again, using a batch size of 1024 and the ReLU activation function, we saw that after adding a dropout layer with a 25% chance of dropping a node after 20 epochs, we got a training accuracy of 98.12% and a test accuracy of 98.36%. This is a large improvement over the 97.47% accuracy we initially received with the same model but without dropout. 

<br>

## Part 3
---
### Implement It!

<br>

Below you can see the duckiebots detecting numbers as they make their way around the track.

<br>

<iframe width="560" height="315" src="https://www.youtube.com/embed/l94tln9xoq8" title="YouTube video player" frameborder="0" allow="accelerometer; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>
<figcaption align = "center"><b>Vid.1: Duckiebot detecting a number</b></figcaption>

<br>

<iframe width="560" height="315" src="https://www.youtube.com/embed/oN4bJrjkix4" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>
<figcaption align = "center"><b>Vid.2: Terminal view of number detection</b></figcaption>

<br>

**What was your strategy for implementing number detection using ML?**

Our strategy was to take repeated predictions of the numbers. Our bot would traverse the town scanning all of the numbers; once a prediction on a tag was made, it would be stored for the duration of the whole program. Once three predictions of the same value was made on the same tag, that tag was marked as complete and would not be scanned again. Once all 10 tags were marked as complete, our program would shut down. Although this did increase our overall run time, it gave us the best shot at correctly identifying all of the tags.

<br>

**How well did your implemented strategy work? Was it reliable? In what situations did it perform poorly?**

Our implemented strategy worked fairly well. Our strategy was unreliable at some points due to the angle of the image it was seeing. Since the MNIST data set has most numbers centred and at no angle, it was harder for our duckiebot to detect the number when it was sitting at a 45$\degree$ from the camera vs dead on. There were also times where the lane follow could cause the bot to kick in a direction which would then interfere when a read of the image was being taken. However, overall the system was fairly reliable. 

<br>

<iframe width="560" height="315" src="https://www.youtube.com/embed/cNec6H42nOs" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>
<figcaption align = "center"><b>Vid.3: Blooper</b></figcaption>



## References
---
[Lab Manual](https://eclass.srv.ualberta.ca/pluginfile.php/9335412/mod_resource/content/1/Exercise%205.pdf)

[dt-core Library](https://github.com/duckietown/dt-core)

[CMPUT 412 Cheat Sheet](https://docs.google.com/document/d/1bQfkR_tmwctFozEZlZkmojBZHkegscJPJVuw-IEXwI4/edit#)

[Backpropagation Step by Step](https://hmkcode.com/ai/backpropagation-step-by-step/)

