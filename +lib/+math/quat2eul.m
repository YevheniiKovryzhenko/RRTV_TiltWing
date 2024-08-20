function eul = quat2eul(quat,squence)
out = quat2eul(quat, squence);
if any(isnan(out))
    eul = zeros(1,3);
else
    eul = [out(1), out(2), out(3)];
end
end

