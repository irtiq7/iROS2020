function [y1, y2] = calculate_rmse(aa,bb,true_toe, N)

xx = length(bb);
mm = zeros(size(bb));
nn = zeros(size(aa));

total1 = 0;
total2 = 0;

mm = (bb - true_toe).^2;
nn = (aa - true_toe).^2;

mean_mm = mean(mm);
mean_nn = mean(nn);



y1 = sqrt(mean_mm);
y2 = sqrt(mean_nn);
end