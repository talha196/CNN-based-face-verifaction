function [] = CNN_face(net,im)
% show the classification result

im_ = single(im) ; % note: 255 range
im_ = imresize(im_, net.meta.normalization.imageSize(1:2)) ;
im_ = bsxfun(@minus,im_,net.meta.normalization.averageImage) ;
res = vl_simplenn(net, im_);
scores = squeeze(gather(res(end).x));
[bestScore, best] = max(scores);
figure(1) ; clf ; imagesc(im); axis equal off ;
title(sprintf('%s (%d), score %.3f',...
              net.meta.classes.description{best}, best, bestScore), ...
      'Interpreter', 'none');

end