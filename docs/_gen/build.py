# -*- coding: utf-8 -*-
import os
import common
import part1
import part2
import part3
import part4
import part5
import part6

part1.render()
part2.render()
part3.render()
part4.render()
part5.render()
part6.render()

common.doc.save(common.OUT)
print("SAVED:", common.OUT)
print("SIZE :", os.path.getsize(common.OUT))