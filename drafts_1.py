
# plan = [0, 1, 2]
# plan_2 = plan[::-1]
# plan_2.append(10)
# print(plan)
# print(plan_2)

for i in range(3):
    print(i)
plan = [0, 1, 2]
i_t = 2

if len(plan) > i_t:
    plan_before = plan[:i_t+1]
    plan_after = plan[i_t+1:]

    print(plan_before)
    print(plan_after)