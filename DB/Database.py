import atexit
import sqlite3


class Database:

	def __init__(self):
		self._conn = sqlite3.connect('../mySite/db.sqlite3')
		self._c = self._conn.cursor()
		atexit.register(self.closeConn)

	def selectBlock(self, table: str, value=None) -> list:
		if value is None:
			self._c.execute('SELECT * FROM {}'.format(table))
		else:
			self._c.execute('SELECT * FROM "{}" WHERE blkNo=?'.format(table), (value,))
		return self._c.fetchall()

	def closeConn(self):
		self._conn.close()
		print("Conn closed")


"""Sample driver code"""
# db = Database()
# print(db.selectBlock("BLOCKS"))
# print(db.selectBlock("BLOCKS", 3))
