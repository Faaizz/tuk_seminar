function or_ang = compute_or_angle(movement)
% Compute angle between fixed-earth axis and a fixed frame attached to the
% robot

% Treshold
tres= 0.001;

y_diff= movement(1);
x_diff= movement(2);

if( (abs(y_diff) > tres) && (abs(x_diff) > tres) )
   
    or_ang= atan2(x_diff, y_diff);
   
else
    
    if(abs(y_diff) <= tres)
       if(x_diff > 0)
           or_ang= pi/2;
       else
           or_ang= -pi/2;
       end
    end
    
    if(abs(x_diff) <= tres)
        if(y_diff > 0)
            or_ang= 0;
        else
            or_ang= pi;
        end
    end
   
end

end

