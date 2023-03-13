# Exercise 4 Report

## TL;DR

Phew! A small lab so we could catch up and string some concepts together things together. In this lab, we implemented autonomous safe tailing behavior for our Duckiebot.
<br>

## Part 1
---
### Follow the Leader

<br>

Below you can see a video of the Duckiebot driving safely behind another Duckiebot. It follow the rules of the road like stopping at the red lines and using a turn signal. If the follower gets too close to the leader, it will stop until a safe distance is observed. 

<iframe width="560" height="315" src="" title="YouTube video player" frameborder="0" allow="accelerometer; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>
<figcaption align = "center"><b>Vid.1: Video for Autonomous Driving</b></figcaption>

<br>

**What strategies were implemented for maintaining a safe driving distance and avoiding collisions?**

Our strategy to keep a safe distance was fairly simple. Given to us was node that detected the distance based on image seen in fig. 1. Using this node, we were able to detect when our Duckiebot got arbitrarily too close to the leader bot and if so, stopped our bot immediately. Currently, no collision avoidance was been used.

<br>

![](figure goes here)

<br>

**How well did your implemented strategy work?**

Our implemented strategy worked ENTER SOME STUFF HERE ABOUT IT. We used our original lane following node to follow within the lines and then the distance detection node to make sure we don't rear-end the leader bot. When the leader bot would turn, we would detect gradual vs sharp turns. 

If the turn was gradual, we assumed it was still following the lane, and we also continued to lane follow. If the turn was sharp we knew it was likely at an intersection. Therefore, we would need to stop and signal. Once that was complete, we turned off our lane following, manually spun the bot 90<span>&#176;</span> to match the leader bot, and turned lane following back on. 

<br>

**Was it reliable?**

answer

<br>

**In what situations did it perform poorly?**

<br>


## References
---
[Lab Manual](https://eclass.srv.ualberta.ca/pluginfile.php/9319824/mod_resource/content/1/Exercise%204.pdf)


[dt-core Library](https://github.com/duckietown/dt-core)

[CMPUT 412 Cheat Sheet](https://docs.google.com/document/d/1bQfkR_tmwctFozEZlZkmojBZHkegscJPJVuw-IEXwI4/edit#)

[Duckietown Docs](https://docs.duckietown.org/daffy/)

