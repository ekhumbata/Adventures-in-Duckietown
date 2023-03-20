# Exercise 4 Report

## TL;DR

In this "catch up" lab we aimed to string some concepts together including lane following, autonomous safe taliling behavior and PID. In Matt Taylor's words: "This should be a fairly easy lab; should only take about three hours." Spoiler alert, it did not.  :D
<br>

## Follow the Leader
---

<br>

**What strategies were implemented for maintaining a safe driving distance and avoiding collisions?**

Our strategy to keep a safe distance was fairly simple. Given to us was node that detected the distance based on image seen in fig. 1. Using this node, we were able to detect when our Duckiebot got arbitrarily too close to the leader bot and if so, stopped our bot immediately. Currently only emergency stop was used, no collision avoidance was implemented.

<br>

![](../docs/assets/images_ex4/backplate.jpg)
<figcaption align = "center"><b>Fig.1: Backplate</b></figcaption>

<br>

**How well did your implemented strategy work?**

Our implemented strategy didn't work too great. We used our original lane following node to follow within the lines and then the distance detection node to make sure we don't rear-end the leader bot. When the leader bot would turn, we wanted to detect gradual vs sharp turns. 

This worked fairly well.

If the turn was gradual, we assumed it was still following the lane, and we also continued to lane follow. If the turn was sharp we knew it was likely at an intersection. Therefore, we would need to stop and signal. Once that was complete, we turned off our lane following, manually spun the bot 90<span>&#176;</span> to match the leader bot, and turned lane following back on. 

This didn't really work.

We got a lot of the items working seperately but couldn't really connect all of them together. We were able to do the following tasks individually:
- Lane following and stopping at the intersections (vid.1 & vid.2)
   - We detected the HSV color values from the red line on the mat and color masked them to filter out any other values. We then used that to determine where we need to execute a stop for the Duckiebot based on the y value of the contour.
   - We used the same lane following node that we did in [Lab 3](https://ekhumbata.github.io/Adventures-in-Duckietown/exercise3.html)
- Turning at an intersection while using LED turn signals (vid.3 & vid.4)
  - To set an angle for the bot we turned off the PID for a short time period and set the angle for the bot to point in a general direction. This brought the correct lane into the FOV in order for us to lane follow off of it.
- Keeping a safe distance behind, while following, the leader bot (vid.5)
  - To keep a safe distance we detected the back plate of the leader bot using the given code which gave us a distance to the bot. After subscribing to the publisher that gave us this valuel. In this callback we set the speed to 0 if the distance was sufficiently small.

Our goal is to update this in the future! We want to substantially upgrade our PID, come up with a better way to track the follower, and implement our turning feature all in one. 

<br>

<iframe src="https://drive.google.com/file/d/1wM1c1zhfyDYZtvnw07niR6xsxBLVSRlx/preview" width="640" height="480" allow="autoplay"></iframe>
<figcaption align = "center"><b>Vid.1:  Lane Following with Stopping at Intersection</b></figcaption>


<iframe src="https://drive.google.com/file/d/1nImmlUqVY7zPiuWZL_mokyNooZOH63QS/preview" width="640" height="480" allow="autoplay"></iframe>
<figcaption align = "center"><b>Vid.2: Stop at intersection</b></figcaption>

<br>

<iframe src="https://drive.google.com/file/d/1HztFwfjgXHBxJHxQBEG96189bhxyHaUB/preview" width="640" height="480" allow="autoplay"></iframe>
<figcaption align = "center"><b>Vid.3: Turn left at intersection</b></figcaption>

<br>

<iframe src="https://drive.google.com/file/d/1zErb8ByrFEqwEGmMPYZekNE_9i3sOsZz/preview" width="640" height="480" allow="autoplay"></iframe>
<figcaption align = "center"><b>Vid.4: Turn right at intersection</b></figcaption>

<br>

<iframe src="https://drive.google.com/file/d/1ZdyzvKPcw8L1zYhp9EV8dT7MApy3kWI0/preview" width="640" height="480" allow="autoplay"></iframe>
<figcaption align = "center"><b>Vid.5: Leader bot detection</b></figcaption>


<br>

**Was it reliable?**

It wasn't very reliable. Our biggest issue came from detecting the image seen in fig.1 due to network issues. When we were building the project on the duckiebot with few people on the 5GHZ network, everything worked a lot smoother. When detecting the back pattern, we ran into a lot of problems with our PID working as accurately as we hoped.

Also, due to these issues, we sometimes ran into problems with picking up the leader bot back plate detection, however this usually happened less than our PID going awry.


<br>

**In what situations did it perform poorly?**

To be honest, our implementation worked fairly well when looked at each part individually. When we started stringing things together, thats when we had problems. The poorest situation was on the turns, we tried to view the turn with object detection and then turn off lane following and PID, spin with dead reckoning, and turn PID lane following back on. It was the poorest situation by far. 

The PID for following the leader bot also didn't work fantastic. The follower bot was *okay* at following the leader when it turned slightly. We didn't implement variable speed but we plan to use implement by relating the distance to the back plate to the speed required.

<br>


## References
---
[Lab Manual](https://eclass.srv.ualberta.ca/pluginfile.php/9319824/mod_resource/content/1/Exercise%204.pdf)

[dt-core Library](https://github.com/duckietown/dt-core)

[CMPUT 412 Cheat Sheet](https://docs.google.com/document/d/1bQfkR_tmwctFozEZlZkmojBZHkegscJPJVuw-IEXwI4/edit#)

[Duckietown Docs](https://docs.duckietown.org/daffy/)

[HSV Colour Picker](https://github.com/botforge/ColorTrackbar/blob/master/HSV%20Trackbar.py)

