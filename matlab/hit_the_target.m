function ends=hit_the_target(newpos,goal,Metric)
if (abs(newpos.x-goal(1))+abs(newpos.y-goal(2)) < Metric)
    ends = true;
else
    ends = false;
end