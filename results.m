acc;

fntSz = 16;

subplot(3, 1, 1);
plot(S, H, 'LineWidth', 2);
set(gca, 'xlim', [S(1), S(end)], 'FontSize', fntSz);
grid on;
title('Профиль дороги', 'FontSize', fntSz);
ylabel('Высота, м', 'FontSize', fntSz);

subplot(3, 1, 2);
plot(S, V1, S, V2, ':r', 'LineWidth', 2);
set(gca, 'xlim', [S(1), S(end)], 'FontSize', fntSz);
grid on;
title('Скорость движения', 'FontSize', fntSz);
legend('адаптивный КК', 'обычный КК', 'FontSize', fntSz);
ylabel('Скорость, км/ч', 'FontSize', fntSz);

subplot(3, 1, 3);
plot(S, F1, S, F2, ':r', 'LineWidth', 2);
set(gca, 'xlim', [S(1), S(end)], 'FontSize', fntSz);
grid on;
title('Объемный расход топлива', 'FontSize', fntSz);
legend('адаптивный КК', 'обычный КК', 'FontSize', fntSz);
ylabel('Расход, л', 'FontSize', fntSz);
xlabel('Перемещение, м', 'FontSize', fntSz);