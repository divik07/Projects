function Ts = DH2transforms(DHtab)

numT = length(DHtab(:,1)); % number of transforms
Ts = cell(numT, 2);
% transforms will hold the transformation matrices
% first entry is the transform from frame to frame (1 is the transform from base to 0)
% second entry is the transform from base up to this frame
% first entry:          1   2   3   4   5   6
% transform to frame:   0   1   2   3   4   T

oldT = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];

for i = 1:numT
    alpha_im1 = DHtab(i,1);
    a_im1 = DHtab(i,2);
    d_i = DHtab(i,3);
    theta_i = DHtab(i,4);

    % calculate the transform
    thisT = LTOA_p( alpha_im1, a_im1, d_i, theta_i);

    if i ~= numT
        fprintf('\n\n\n%i \n   T\n%i\n\n', i-1, i)
    else
        fprintf('\n\n\n%i \n   T\nN\n\n', i-1)
    end
    disp(thisT)


    oldT = oldT*thisT;

    Ts{i, 1} = thisT;
    Ts{i, 2} = oldT;
end

fprintf('\n\n\n0 \n   T\nN\n\n')
disp(simplify(oldT))